#include "control_grampc/mpcc_grampc_node.hpp"

using std::placeholders::_1;

MPCCGrampcNode::MPCCGrampcNode()
        : Node("mpcc_grampc_node"), x_(0.0), y_(0.0), yaw_(0.0), v_(0.0), prev_steer_(0.0), prev_throttle_(0.0), t(0.0)
    {
        // Load path from CSV file
        std::string csv_file = this->declare_parameter<std::string>("path_csv",
                                                                    "/home/ammar/ros2_ws/src/global-planning/outputs/map5/traj_race_cl.csv");
        path_ = std::make_shared<mpcc::Path>(mpcc::load_path_from_csv(csv_file));

        if (path_ && path_->total_length() > 1e-3)
        {
            RCLCPP_INFO(this->get_logger(), "Path loaded successfully: %s, length: %f", csv_file.c_str(),
                        path_->total_length());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path from: %s", csv_file.c_str());
        }

        // ROS interfaces - using individual subscribers for better robustness
        ips_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/autodrive/f1tenth_1/ips", 10,
            std::bind(&MPCCGrampcNode::ipsCallback, this, std::placeholders::_1));

        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/speed", 10,
            std::bind(&MPCCGrampcNode::speedCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu", 10,
            std::bind(&MPCCGrampcNode::imuCallback, this, std::placeholders::_1));

        // Create a timer for periodic control updates (20Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MPCCGrampcNode::controlTimerCallback, this));

        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);

        // Initialize parameter array for GRAMPC vehicle model
        // Improved tuning for stable and smooth control:
        param_[0] = 0.32;  // L (wheelbase) - actual F1/10 wheelbase
        param_[1] = 3.0;   // v_ch (characteristic velocity) - reduced for better low-speed behavior
        param_[2] = 10.0;  // w_x (x position weight) - balanced for path following
        param_[3] = 10.0;  // w_y (y position weight) - balanced for path following
        param_[4] = 8.0;   // w_theta (heading weight) - moderate for stability
        param_[5] = 15.0;  // w_kappa (curvature weight) - moderate to allow necessary turns
        param_[6] = 2.0;   // w_v (velocity weight) - low to allow speed variations
        param_[7] = 0.0;   // w_x_T (terminal x weight)
        param_[8] = 50.0;  // w_y_T (terminal y weight) - reduced
        param_[9] = 50.0;  // w_theta_T (terminal heading weight) - reduced
        param_[10] = 30.0; // w_kappa_T (terminal curvature weight) - reduced
        param_[11] = 2.0;  // w_v_T (terminal velocity weight) - reduced
        param_[12] = 2.0;  // w_u0 (acceleration effort weight) - reduced since bounds are tighter
        param_[13] = 10.0; // w_u1 (steering rate effort weight) - reduced since bounds are tighter

        // Store important vehicle parameters for easy access
        L_ = param_[0];
        delta_max_ = M_PI / 6; // Maximum steering angle in radians (30 degrees)
        v_max_ = 5.0;          // Maximum velocity

        // Initialize GRAMPC with safety checks
        grampc_ = nullptr;
        grampc_init(&grampc_, (void *)param_);
        // RCLCPP_INFO(this->get_logger(), "GRAMPC init success");

        if (!grampc_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GRAMPC solver");
            return;
        }

        if (!grampc_->param || !grampc_->opt || !grampc_->sol)
        {
            RCLCPP_ERROR(this->get_logger(), "GRAMPC not properly initialized");
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "GRAMPC pointer and structures are valid");

        // Set problem dimensions - these MUST match exactly with ocp_dim() in mpcc_model.c
        grampc_->param->Nx = NX; // [x, y, theta, kappa, v]
        grampc_->param->Nu = NU; // [acceleration, steering_rate]
        grampc_->param->Nh = NH; // velocity and curvature constraints
        grampc_->param->Np = 0;
        grampc_->param->Ng = 0;
        grampc_->param->NhT = 0;
        grampc_->param->NgT = 0;

        // Set initial state and control before running
        double x0_init[NX] = {0.0, 0.0, 0.0, 0.0, 0.5};   // 5D state [x, y, theta, kappa, v] - start with minimum forward speed
        double xref_init[NX] = {0.0, 0.0, 0.0, 0.0, 1.0}; // [x, y, theta, kappa, v]

        double u0_init[NU] = {0.0, 0.0};     // [acceleration, steering_rate] - small initial acceleration
        double umin[NU] = {-0.1, -M_PI / 6}; // acceleration_min=-0.1 (small reverse for maneuvering), steering_rate_min - F1/10 physical limits
        double umax[NU] = {1.0, M_PI / 6};   // acceleration_max, steering_rate_max - F1/10 physical limits

        // Safety check before setting parameters
        if (grampc_ && grampc_->param)
        {
            grampc_setparam_real_vector(grampc_, "x0", x0_init);
            grampc_setparam_real_vector(grampc_, "xdes", xref_init);
            grampc_setparam_real_vector(grampc_, "u0", u0_init);
            grampc_setparam_real_vector(grampc_, "umin", umin);
            grampc_setparam_real_vector(grampc_, "umax", umax);

            grampc_setparam_real(grampc_, "Thor", 1.5); // Prediction horizon - shorter for better responsiveness
            grampc_setparam_real(grampc_, "t0", 0.0);   // Initial time

            // CRITICAL: Set solver options explicitly (optimized for stability)
            grampc_setopt_int(grampc_, "Nhor", 20);       // Number of discretization steps - reduced for faster computation
            grampc_setopt_int(grampc_, "MaxGradIter", 5); // Maximum gradient iterations - reduced for faster computation
            ctypeRNum ConstraintsAbsTol[1] = {1e-4};      // Tighter constraint tolerance for better enforcement
            grampc_setopt_real_vector(grampc_, "ConstraintsAbsTol", ConstraintsAbsTol);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot set GRAMPC parameters - structures not initialized");
        }
    }

    void MPCCGrampcNode::ipsCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        x_ = msg->x;
        y_ = msg->y;
        has_ips_data_ = true;
        last_ips_time_ = this->now();
    }

    void MPCCGrampcNode::speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // Update speed data with low-pass filter
        v_ = 0.7 * v_ + 0.3 * msg->data;
        has_speed_data_ = true;
        last_speed_time_ = this->now();
    }

    void MPCCGrampcNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract yaw from IMU quaternion (much more accurate than GPS displacement)
        const auto &q = msg->orientation;

        // Convert quaternion to yaw angle (Z-axis rotation)
        double yaw_from_imu = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        // Apply light filtering for IMU yaw
        if (has_imu_data_)
        {
            // Wrap angle difference to [-π, π]
            double dyaw = std::atan2(std::sin(yaw_from_imu - yaw_), std::cos(yaw_from_imu - yaw_));
            yaw_ += 0.8 * dyaw;

            // Keep yaw_ in [-π, π]
            yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));
        }
        else
        {
            yaw_ = yaw_from_imu; // First measurement
        }

        has_imu_data_ = true;
        last_imu_time_ = this->now();
    }

    void MPCCGrampcNode::controlTimerCallback()
    {
        // Update timing
        const double now_sec = this->now().seconds();
        if (last_time_ > 0.0)
        {
            dt_ = now_sec - last_time_;
            grampc_setparam_real(grampc_, "dt", dt_);
        }
        else
        {
            dt_ = 0.05; // Default 50ms for 20Hz timer
            grampc_setparam_real(grampc_, "dt", dt_);
        }
        last_time_ = now_sec;

        // Check if we have all required data and it's recent enough (within 500ms)
        auto now = this->now();
        auto max_age = rclcpp::Duration::from_nanoseconds(500000000); // 500ms

        bool data_fresh = true;
        if (!has_ips_data_ || (now - last_ips_time_) > max_age)
        {
            data_fresh = false;
        }
        if (!has_speed_data_ || (now - last_speed_time_) > max_age)
        {
            data_fresh = false;
        }
        if (!has_imu_data_ || (now - last_imu_time_) > max_age)
        {
            data_fresh = false;
        }

        if (!data_fresh)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for fresh sensor data - IPS:%d, Speed:%d, IMU:%d",
                                 has_ips_data_, has_speed_data_, has_imu_data_);
            return;
        }

        // Run the control loop
        controlLoop();
    }

    void MPCCGrampcNode::initializePathPosition()
    {
        // Initialize with closest waypoint on first run
        Eigen::Vector2d vehicle_pos(x_, y_);
        current_path_idx_ = 0;
        double min_dist = (vehicle_pos - path_->getWaypoint(0)).norm();
        for (size_t i = 1; i < path_->getWaypointCount(); ++i)
        {
            double dist = (vehicle_pos - path_->getWaypoint(i)).norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                current_path_idx_ = i;
            }
        }
        current_s_ = path_->getArcLength(current_path_idx_);
        path_initialized_ = true;
    }

    void MPCCGrampcNode::controlLoop()
    {
        // Safety check: Ensure we have a valid path
        if (!path_ || path_->total_length() < 1e-3)
        {
            RCLCPP_WARN(this->get_logger(), "No valid path found, skipping control loop");
            return;
        }

        if (!path_initialized_)
        {
            // Update current arc length position based on current waypoint since we can start from anywhere
            initializePathPosition();
        }

        // Find current waypoint using arc length and get next waypoint
        size_t nextIdx = path_->findNextWaypointIdx(current_s_);
        RCLCPP_INFO(this->get_logger(), "Current State: [next_idx=%zu]",
                    nextIdx);
        // Update current arc length based on progress
        current_s_ = path_->getArcLength(nextIdx);

        // Get target waypoint and reference states
        Eigen::Vector2d target_point = path_->getWaypoint(nextIdx);
        double target_heading = path_->getHeading(nextIdx);
        double target_curvature = path_->getCurvature(nextIdx);
        double ref_speed = std::min(4.0, std::max(1.5, v_ + 0.3));

        // Current state and target state
        kappa_ = std::tan(steering_angle_) / L_;
        std::vector<double> current_state = {x_, y_, yaw_, kappa_, v_};
        std::vector<double> target_state = {target_point.x(), target_point.y(), target_heading, target_curvature, ref_speed};
        RCLCPP_INFO(this->get_logger(), "=== STATE DEBUG ===");
        RCLCPP_INFO(this->get_logger(), "Current State: [x=%.3f, y=%.3f, yaw=%.3f, kappa=%.3f, v=%.3f]",
                    x_, y_, yaw_, kappa_, v_);
        RCLCPP_INFO(this->get_logger(), "Target State:  [x=%.3f, y=%.3f, yaw=%.3f, kappa=%.3f, v=%.3f]",
                    target_point.x(), target_point.y(), target_heading, target_curvature, ref_speed);
        RCLCPP_INFO(this->get_logger(), "Path progress: current_s=%.3f, next_idx=%zu",
                    current_s_, nextIdx);
        RCLCPP_INFO(this->get_logger(), "==================");

        // Set current state as initial condition
        grampc_setparam_real_vector(grampc_, "x0", current_state.data());
        grampc_setparam_real_vector(grampc_, "xdes", target_state.data());

        double steer_cmd = 0.0;
        double throttle_cmd = 0.0;

        // debugging
        auto solver_start = std::chrono::high_resolution_clock::now();

        grampc_run(grampc_);

        // Check for any GRAMPC solver issues - treat all status > 0 as errors
        if (grampc_->sol->status > 0)
        {
            RCLCPP_INFO(this->get_logger(), "GRAMPC solver error with status: %d", grampc_->sol->status);

            // Use previous commands when solver fails
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
        }
        else
        {
            double acceleration = grampc_->sol->unext[0]; // u[0] = acceleration [m/s^2]
            double kappa_dot = grampc_->sol->unext[1];    // u[1] = steering_rate (curvature rate) [1/s]

            // Apply rate limiting and filtering for smoother control
            double max_accel_change = 0.5 * dt_; // Max acceleration change per timestep (more conservative)
            double max_steer_change = 1.0 * dt_; // Max steering rate change per timestep

            acceleration = std::max(0.0, std::min(1.0, acceleration)); // Clamp to F1/10 acceleration limits (no reverse)

            // Integration: kappa = kappa_prev + kappa_dot * dt
            kappa_ += kappa_dot * dt_;

            // Convert curvature to steering angle: delta = atan(kappa * L)
            double target_steering_angle = std::atan(kappa_ * L_);

            // Limit steering angle to physical constraints
            target_steering_angle = std::max(-M_PI / 6, std::min(M_PI / 6, target_steering_angle)); // ±30 degrees (π/6 rad)

            throttle_cmd = acceleration;
            steer_cmd = target_steering_angle;
        }

        auto solver_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(solver_end - solver_start);

        // Publish steering command
        auto s_msg = std_msgs::msg::Float32();
        s_msg.data = static_cast<float>(steer_cmd);
        steering_pub_->publish(s_msg);

        // Publish throttle command
        auto t_msg = std_msgs::msg::Float32();
        t_msg.data = static_cast<float>(throttle_cmd);
        throttle_pub_->publish(t_msg);

        // Update previous commands for next iteration
        prev_steer_ = steer_cmd;
        prev_throttle_ = throttle_cmd;

        // Update time for GRAMPC integration
        t += dt_;
        grampc_setparam_real(grampc_, "t0", t);
    }

int main(int argc, char *argv[])
{
    std::cout << "Starting GRAMPC MPCC Node..." << std::endl;

    rclcpp::init(argc, argv);
    std::cout << "ROS2 initialized successfully" << std::endl;

    try
    {
        std::cout << "Creating MPCCGrampcNode..." << std::endl;
        auto node = std::make_shared<MPCCGrampcNode>();
        std::cout << "Node created successfully" << std::endl;

        std::cout << "Starting ROS2 spin..." << std::endl;
        rclcpp::spin(node);
        std::cout << "ROS2 spin completed" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "Unknown exception caught" << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    std::cout << "ROS2 shutdown completed" << std::endl;
    return 0;
}
