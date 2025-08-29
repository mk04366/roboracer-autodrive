#include "geometry_msgs/msg/point.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "control_grampc/path_utils.hpp"
#include "control_grampc/mpcc_model.h"
extern "C"
{
#include "grampc.h"
}
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>  // For M_PI and std::abs
#include <limits> // For std::numeric_limits

using std::placeholders::_1;

#define NX 5
#define NU 2
#define NC 1
#define NP 0

// Helper function to wrap angle to [-pi, pi]
double wrap_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

class MPCCGrampcNode : public rclcpp::Node
{
public:
    MPCCGrampcNode()
        : Node("mpcc_grampc_node"), x_(0.0), y_(0.0), yaw_(0.0), v_(0.0), s_(0.0), prev_steer_(0.0), prev_throttle_(0.0)
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

        // ROS interfaces
        ips_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/autodrive/f1tenth_1/ips", 30,
                                                                        std::bind(&MPCCGrampcNode::ipsCallback, this, _1));

        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);

        // Initialize parameter array for GRAMPC vehicle model
        // Following the vehicle problem structure:
        param_[0] = 0.33;  // L (wheelbase)
        param_[1] = 10.0;  // v_ch (characteristic velocity)
        param_[2] = 5.0;   // w_x (x position weight)
        param_[3] = 5.0;   // w_y (y position weight)
        param_[4] = 10.0;  // w_theta (heading weight)
        param_[5] = 1.0;   // w_kappa (curvature weight)
        param_[6] = 1.0;   // w_v (velocity weight)
        param_[7] = 10.0;  // w_x_T (terminal x weight)
        param_[8] = 10.0;  // w_y_T (terminal y weight)
        param_[9] = 5.0;   // w_theta_T (terminal heading weight)
        param_[10] = 1.0;  // w_kappa_T (terminal curvature weight)
        param_[11] = 1.0;  // w_v_T (terminal velocity weight)
        param_[12] = 0.1;  // w_u0 (acceleration effort weight)
        param_[13] = 0.05; // w_u1 (steering rate effort weight)
        param_[14] = 5.0;  // v_max (maximum velocity for constraints)

        // Store important vehicle parameters for easy access
        L_ = param_[0];
        delta_max_ = 0.4;
        v_max_ = param_[14];

        // Initialize GRAMPC with safety checks
        grampc_ = nullptr;
        grampc_init(&grampc_, (void *)param_);
        RCLCPP_INFO(this->get_logger(), "GRAMPC init success");

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

        RCLCPP_INFO(this->get_logger(), "GRAMPC pointer and structures are valid");

        // Set problem dimensions - these MUST match exactly with ocp_dim() in mpcc_model.c
        grampc_->param->Nx = NX; // [x, y, theta, kappa, v] - 5D vehicle state
        grampc_->param->Nu = NU; // [acceleration, steering_rate]
        grampc_->param->Ng = 0;
        grampc_->param->Nh = NC; // velocity constraint
        grampc_->param->Np = NP;
        grampc_->param->NhT = 0;
        grampc_->param->NgT = 0;

        // Set initial state and control before running
        std::vector<double> x0_init[NX] = {0.0, 0.0, 0.0, 0.0, 0.0};   // 5D state [x, y, theta, kappa, v]
        std::vector<double> xref_init[NX] = {0.0, 0.0, 0.0, 0.0, 1.0}; // [x, y, theta, kappa, v]

        std::vector<double> u0_init[NU] = {0.0, 0.0}; // [acceleration, steering_rate]
        double umin[NU] = {-1.0, -1.0};               // acceleration_min, steering_rate_min
        double umax[NU] = {1.0, 1.0};                 // acceleration_max, steering_rate_max

        // Safety check before setting parameters
        if (grampc_ && grampc_->param)
        {
            grampc_setparam_real_vector(grampc_, "x0", x0_init);
            grampc_setparam_real_vector(grampc_, "xdes", xref_init.data());

            grampc_setparam_real_vector(grampc_, "u0", u0_init);
            grampc_setparam_real_vector(grampc_, "umin", umin);
            grampc_setparam_real_vector(grampc_, "umax", umax);

            grampc_setparam_real(grampc_, "Thor", 1);  // Prediction horizon
            grampc_setparam_real(grampc_, "dt", 0.01); // Discretization time step
            grampc_setparam_real(grampc_, "t0", 0.0);  // Initial time

            // CRITICAL: Set solver options explicitly (following Vehicle example)
            grampc_setopt_int(grampc_, "Nhor", 20);        // Number of discretization steps
            grampc_setopt_int(grampc_, "MaxGradIter", 10); // Maximum gradient iterations

            RCLCPP_INFO(this->get_logger(), "GRAMPC parameters set successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot set GRAMPC parameters - structures not initialized");
        }

        // Skip penalty estimation for now since we disabled constraints
        RCLCPP_INFO(this->get_logger(), "GRAMPC setup completed successfully");

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&MPCCGrampcNode::controlLoop, this));
    }

private:
    void ipsCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        const double now_sec = this->now().seconds();
        const double dx = msg->x - last_x_;
        const double dy = msg->y - last_y_;
        const double dt = now_sec - last_time_;

        x_ = msg->x;
        y_ = msg->y;

        if (has_prev_fix_ && dt > 1e-3)
        {
            const double new_yaw = std::atan2(dy, dx);
            // Simple unwrap
            const double dyaw = std::atan2(std::sin(new_yaw - yaw_), std::cos(new_yaw - yaw_));
            yaw_ += 0.5 * dyaw;

            const double speed = std::sqrt(dx * dx + dy * dy) / dt;
            v_ = 0.7 * v_ + 0.3 * speed; // low-pass filter
        }
        else
        {
            // First fix or invalid dt
            yaw_ = yaw_;
            v_ = 0.0;
            has_prev_fix_ = true;
        }

        last_x_ = x_;
        last_y_ = y_;
        last_time_ = now_sec;
    }

    void controlLoop()
    {
        // Safety check: Ensure GRAMPC is properly initialized
        if (!grampc_ || !grampc_->param || !grampc_->sol)
        {
            RCLCPP_ERROR(this->get_logger(), "GRAMPC not initialized, skipping control loop");
            return;
        }

        // Safety check: Ensure we have a valid path
        if (!path_ || path_->total_length() < 1e-3)
        {
            RCLCPP_WARN(this->get_logger(), "No path found, skipping control loop");
            return;
        }

        // Safety check: Ensure we have received IPS data
        if (!has_prev_fix_)
        {
            RCLCPP_INFO(this->get_logger(), "No IPS data received yet, using default position");
            x_ = 0.0;
            y_ = 0.0;
            yaw_ = 0.0;
            v_ = 0.0;
            has_prev_fix_ = true;
        }

        // Update progress state based on vehicle's position along the path
        // Find the closest point on the path to current vehicle position
        Eigen::Vector2d vehicle_pos(x_, y_);
        double min_dist = std::numeric_limits<double>::max();
        double best_s = s_;

        // Sample along the path to find the closest point
        for (double test_s = 0.0; test_s <= path_->total_length(); test_s += 0.2)
        {
            Eigen::Vector2d path_point = path_->interpolate(test_s);
            double dist = (vehicle_pos - path_point).norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                best_s = test_s;
            }
        }
        s_ = best_s;

        // Get reference point ahead on the path for path following
        double ref_distance = 1.0; // Look-ahead distance in meters
        double s_ref = std::min(s_ + ref_distance, path_->total_length());

        Eigen::Vector2d ref_pos = path_->interpolate(s_ref);
        double ref_yaw = path_->heading(s_ref);
        double ref_speed = 0.5; // Target speed for path following

        // Update curvature based on current steering angle and velocity
        if (v_ > 0.1)
        {
            kappa_ = std::tan(steering_angle_) / L_;
        }
        else
        {
            kappa_ = 0.0;
        }

        std::vector<double> current_state = {x_, y_, yaw_, kappa_, v_};

        // Desired target state (path following) - 5D [x, y, theta, kappa, v]
        double ref_curvature = path_->curvature(s_ref);
        std::vector<double> target_state = {ref_pos.x(), ref_pos.y(), ref_yaw, ref_curvature, ref_speed};

        // Set current state as initial condition
        grampc_setparam_real_vector(grampc_, "x0", current_state.data());
        grampc_setparam_real_vector(grampc_, "xdes", target_state.data());

        double steer_cmd = 0.0;
        double throttle_cmd = 0.0;

        // Enhanced GRAMPC solver execution with debugging
        auto solver_start = std::chrono::high_resolution_clock::now();

        // Extensive validation before calling grampc_run
        if (!grampc_->sol->xnext || !grampc_->sol->unext)
        {
            RCLCPP_ERROR(this->get_logger(), "GRAMPC solution vectors not allocated");
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
        }
        else if (!grampc_->rws->x || !grampc_->rws->u)
        {
            RCLCPP_ERROR(this->get_logger(), "GRAMPC workspace vectors not allocated");
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
        }
        else
        {
            try
            {
                // Try to run GRAMPC with exception handling
                RCLCPP_INFO(this->get_logger(), "Calling grampc_run...");
                grampc_run(grampc_);
                RCLCPP_INFO(this->get_logger(), "grampc_run completed with status: %d", grampc_->sol->status);
            }
            catch (...)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception during grampc_run");
                steer_cmd = prev_steer_;
                throttle_cmd = prev_throttle_;
                return;
            }
        }

        if (grampc_->sol->status > 0)
        {
            RCLCPP_WARN(this->get_logger(), "GRAMPC solver failed with status %d", grampc_->sol->status);
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
        }
        else
        {
            // Extract control from solution - now [acceleration, steering_rate]
            double acceleration = grampc_->sol->unext[0];  // acceleration [m/s^2]
            double steering_rate = grampc_->sol->unext[1]; // steering rate [rad/s]

            // Convert acceleration to throttle command (simple mapping)
            throttle_cmd = std::max(0.0, std::min(1.0, acceleration / 2.0 + 0.5)); // normalize to [0,1]

            // Integrate steering rate to get steering angle (simple Euler integration)
            double dt = 0.02; // control loop period
            steering_angle_ += steering_rate * dt;
            steering_angle_ = std::max(-delta_max_, std::min(delta_max_, steering_angle_)); // clamp to limits
            steer_cmd = steering_angle_;

            RCLCPP_INFO(this->get_logger(), "MPCC Solution - acceleration: %.3f, steering_rate: %.3f -> throttle: %.3f, steer: %.3f",
                        acceleration, steering_rate, throttle_cmd, steer_cmd);
        }

        auto solver_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(solver_end - solver_start);
        RCLCPP_INFO(this->get_logger(), "GRAMPC solve time: %ld ms", duration.count());

        // Apply control smoothing
        const double alpha_steer = 0.7;
        const double alpha_throttle = 0.8;

        // Smooth the commands
        steer_cmd = alpha_steer * steer_cmd + (1.0 - alpha_steer) * prev_steer_;
        throttle_cmd = alpha_throttle * throttle_cmd + (1.0 - alpha_throttle) * prev_throttle_;

        // Publish steering command
        auto s_msg = std_msgs::msg::Float32();
        s_msg.data = static_cast<float>(steer_cmd);
        steering_pub_->publish(s_msg);

        // Publish throttle command
        auto t_msg = std_msgs::msg::Float32();
        t_msg.data = static_cast<float>(throttle_cmd);
        throttle_pub_->publish(t_msg);

        RCLCPP_INFO(this->get_logger(), "Published commands - steer: %.3f, throttle: %.3f", steer_cmd, throttle_cmd);

        // Update previous commands for next iteration
        prev_steer_ = steer_cmd;
        prev_throttle_ = throttle_cmd;
    }

    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ips_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<mpcc::Path> path_;

    // Vehicle state - 5D [x, y, theta, kappa, v]
    double x_, y_, yaw_, v_, s_;
    double kappa_ = 0.0;          // current curvature
    double steering_angle_ = 0.0; // current steering angle for curvature calculation
    double last_x_ = 0.0, last_y_ = 0.0, last_time_ = 0.0;
    bool has_prev_fix_ = false;

    // Control history for smoothing
    double prev_steer_, prev_throttle_;

    // GRAMPC solver and parameter array
    TYPE_GRAMPC_POINTER(grampc_);
    double param_[15]; // Parameter array for GRAMPC vehicle model

    // Easy access to key parameters
    double L_, delta_max_, v_max_;
};

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
