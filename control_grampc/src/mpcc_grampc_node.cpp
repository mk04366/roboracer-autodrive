#include "geometry_msgs/msg/point.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "control_grampc/path_utils.hpp"
#include "control_grampc/mpcc_model.h"
extern "C"
{
#include "grampc.h"
#include "grampc_mess.h"
}
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include <memory>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>  // For M_PI and std::abs
#include <limits> // For std::numeric_limits
#include <algorithm>
#include <chrono>
#include <cmath>  // For M_PI and std::abs
#include <limits> // For std::numeric_limits

using std::placeholders::_1;

#define NX 5
#define NU 2
#define NC 0
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

        // ROS interfaces - using message filters for synchronization
        ips_sub_.subscribe(this, "/autodrive/f1tenth_1/ips");
        speed_sub_.subscribe(this, "/autodrive/f1tenth_1/speed");
        
        // Create time synchronizer for IPS and speed data
        sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::msg::Point, std_msgs::msg::Float32>>(
            ips_sub_, speed_sub_, 10);
        sync_->registerCallback(&MPCCGrampcNode::synchronizedCallback, this);

        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);

        // Initialize parameter array for GRAMPC vehicle model
        // Following the vehicle problem structure:
        param_[0] = 3;  // L (wheelbase)
        param_[1] = 50.0;  // v_ch (characteristic velocity)
        param_[2] = 1.0;   // w_x (x position weight)
        param_[3] = 1.0;   // w_y (y position weight)
        param_[4] = 1.0;  // w_theta (heading weight)
        param_[5] = 1.0;   // w_kappa (curvature weight)
        param_[6] = 100.0;   // w_v (velocity weight)
        param_[7] = 0.0;  // w_x_T (terminal x weight)
        param_[8] = 1.0;  // w_y_T (terminal y weight)
        param_[9] = 1.0;   // w_theta_T (terminal heading weight)
        param_[10] = 1.0;  // w_kappa_T (terminal curvature weight)
        param_[11] = 100.0;  // w_v_T (terminal velocity weight)
        param_[12] = 0.1;  // w_u0 (acceleration effort weight)
        param_[13] = 0.1; // w_u1 (steering rate effort weight)

        // Store important vehicle parameters for easy access
        L_ = param_[0];
        delta_max_ = 0.4;
        v_max_ = 30.0;  // Set a reasonable default max velocity since we removed velocity constraints

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
        grampc_->param->Nx = NX; // [x, y, theta, kappa, v] - 5D vehicle state
        grampc_->param->Nu = NU; // [acceleration, steering_rate]
        grampc_->param->Ng = 0;
        grampc_->param->Nh = NC; // no inequality constraints (NC = 0)
        grampc_->param->Np = NP;
        grampc_->param->NhT = 0;
        grampc_->param->NgT = 0;

        // Set initial state and control before running
        double x0_init[NX] = {0.0, 0.0, 0.0, 0.0, 0.0};   // 5D state [x, y, theta, kappa, v]
        double xref_init[NX] = {0.0, 0.0, 0.0, 0.0, 1.0}; // [x, y, theta, kappa, v]

        double u0_init[NU] = {0.0, 0.0}; // [acceleration, steering_rate]
        double umin[NU] = {-3.0, -2.0};  // acceleration_min, steering_rate_min - relaxed bounds
        double umax[NU] = {3.0, 2.0};    // acceleration_max, steering_rate_max - relaxed bounds

        // Safety check before setting parameters
        if (grampc_ && grampc_->param)
        {
            grampc_setparam_real_vector(grampc_, "x0", x0_init);
            grampc_setparam_real_vector(grampc_, "xdes", xref_init);
            grampc_setparam_real_vector(grampc_, "u0", u0_init);
            grampc_setparam_real_vector(grampc_, "umin", umin);
            grampc_setparam_real_vector(grampc_, "umax", umax);

            grampc_setparam_real(grampc_, "Thor", 1);  // Prediction horizon
            grampc_setparam_real(grampc_, "dt", 0.01); // Discretization time step
            grampc_setparam_real(grampc_, "t0", 0.0);  // Initial time

            // CRITICAL: Set solver options explicitly (following Vehicle example)
            grampc_setopt_int(grampc_, "Nhor", 20);        // Number of discretization steps
            grampc_setopt_int(grampc_, "MaxGradIter", 10); // Maximum gradient iterations

            // RCLCPP_INFO(this->get_logger(), "GRAMPC parameters set successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot set GRAMPC parameters - structures not initialized");
        }
    }

private:
    void synchronizedCallback(const geometry_msgs::msg::Point::SharedPtr ips_msg,
                             const std_msgs::msg::Float32::SharedPtr speed_msg)
    {
        // Update position data from IPS
        const double now_sec = this->now().seconds();
        const double dx = ips_msg->x - last_x_;
        const double dy = ips_msg->y - last_y_;
        const double dt = now_sec - last_time_;

        x_ = ips_msg->x;
        y_ = ips_msg->y;

        if (has_prev_fix_ && dt > 1e-3)
        {
            const double new_yaw = std::atan2(dy, dx);
            // Simple unwrap
            const double dyaw = std::atan2(std::sin(new_yaw - yaw_), std::cos(new_yaw - yaw_));
            yaw_ += 0.5 * dyaw;
            
            // Calculate curvature from vehicle motion (rate of change of heading)
            const double yaw_rate = dyaw / dt;
            const double current_speed = std::max(0.1, v_); // Avoid division by zero
            kappa_ = yaw_rate / current_speed;
        }
        else
        {
            // First fix or invalid dt
            yaw_ = yaw_;
            kappa_ = 0.0;
            has_prev_fix_ = true;
        }

        last_x_ = x_;
        last_y_ = y_;
        last_time_ = now_sec;

        // Update speed data with low-pass filter
        v_ = 0.7 * v_ + 0.3 * speed_msg->data;

        // Now that we have synchronized data, run the control loop
        controlLoop();
    }

    void controlLoop()
    {
        // Safety check: Ensure we have a valid path
        if (!path_ || path_->total_length() < 1e-3)
        {
            RCLCPP_WARN(this->get_logger(), "No valid path found, skipping control loop");
            return;
        }

        // Safety check: Ensure we have received IPS data
        if (!has_prev_fix_)
        {
            // RCLCPP_INFO(this->get_logger(), "No IPS data received yet, using default position");
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

        // Sample along the path to find the closest point
        for (double test_s = 0.0; test_s <= path_->total_length(); test_s += 0.1) // reduced from 0.2 to 0.1 for better accuracy
        {
            Eigen::Vector2d path_point = path_->interpolate(test_s);
            double dist = (vehicle_pos - path_point).norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                s_ = test_s;
            }
        }

        // Get reference point ahead on the path for path following
        double ref_distance = std::max(0.5, v_ * 0.5); // Dynamic look-ahead based on velocity (0.5-1.5s ahead)
        double s_ref = std::min(s_ + ref_distance, path_->total_length());

        Eigen::Vector2d ref_pos = path_->interpolate(s_ref);
        double ref_yaw = path_->heading(s_ref);
        double ref_speed = std::min(5.0, std::max(1.0, v_ + 0.5)); // Progressive speed increase

        // Current and target state vectors for GRAMPC
        std::vector<double> current_state = {x_, y_, yaw_, kappa_, v_};
        double ref_curvature = path_->curvature(s_ref);

        // Desired target state (path following) - [x, y, theta, kappa, v]
        std::vector<double> target_state = {ref_pos.x(), ref_pos.y(), ref_yaw, ref_curvature, ref_speed};

        // Print the current_state and target_state every time:
        RCLCPP_INFO(this->get_logger(), "Current State: [x=%.3f, y=%.3f, yaw=%.3f, kappa=%.3f, v=%.3f]", 
                    current_state[0], current_state[1], current_state[2], current_state[3], current_state[4]);
        RCLCPP_INFO(this->get_logger(), "Target State:  [x=%.3f, y=%.3f, yaw=%.3f, kappa=%.3f, v=%.3f]", 
                    target_state[0], target_state[1], target_state[2], target_state[3], target_state[4]);
        RCLCPP_INFO(this->get_logger(), "Path Info: s_current=%.3f, s_ref=%.3f, min_dist=%.3f", 
                    s_, s_ref, min_dist);

        // Set current state as initial condition
        grampc_setparam_real_vector(grampc_, "x0", current_state.data());
        grampc_setparam_real_vector(grampc_, "xdes", target_state.data());

        double steer_cmd = 0.0;
        double throttle_cmd = 0.0;

        // debugging
        auto solver_start = std::chrono::high_resolution_clock::now();

        try
        {
            grampc_run(grampc_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception during grampc_run: %s", e.what());
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
            return;
        }

        // Check for any GRAMPC solver issues - treat all status > 0 as errors
        if (grampc_->sol->status > 0)
        {
            // Print status for debugging
            grampc_printstatus(grampc_->sol->status, STATUS_LEVEL_ERROR);
            RCLCPP_WARN(this->get_logger(), "GRAMPC solver failed with status %d - using previous commands", grampc_->sol->status);
            
            // Use previous commands when solver fails
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
        }
        else
        {
            // Extract control solution only when solver succeeds (status == 0)
            double acceleration = grampc_->sol->unext[0];  // acceleration [m/s^2]
            double steering_rate = grampc_->sol->unext[1]; // steering rate [rad/s]

            // Convert acceleration to throttle command
            if (acceleration >= 0.0) {
                throttle_cmd = std::min(1.0, acceleration / 3.0);
            } else {
                throttle_cmd = std::max(0.0, 0.5 + acceleration / 6.0);
            }

            // Convert steering rate to steering angle: delta = atan(kappa * L)
            steering_angle_ = std::atan(kappa_ * L_);
            steering_angle_ = std::max(-delta_max_, std::min(delta_max_, steering_angle_));
            steer_cmd = steering_angle_;

            RCLCPP_INFO(this->get_logger(), "MPCC Solution - acceleration: %.3f, steering_rate: %.3f, kappa: %.3f -> throttle: %.3f, steer: %.3f",
                        acceleration, steering_rate, kappa_, throttle_cmd, steer_cmd);
        }

        auto solver_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(solver_end - solver_start);
        // RCLCPP_INFO(this->get_logger(), "GRAMPC solve time: %ld ms", duration.count());

        // Apply control smoothing
        const double alpha_steer = 0.7;
        const double alpha_throttle = 0.8;

        // Smooth the commands
        // steer_cmd = alpha_steer * steer_cmd + (1.0 - alpha_steer) * prev_steer_;
        // throttle_cmd = alpha_throttle * throttle_cmd + (1.0 - alpha_throttle) * prev_throttle_;

        // Publish steering command
        auto s_msg = std_msgs::msg::Float32();
        s_msg.data = static_cast<float>(steer_cmd);
        RCLCPP_INFO(this->get_logger(), "Steering Command: %.3f", steer_cmd);
        steering_pub_->publish(s_msg);

        // Publish throttle command
        auto t_msg = std_msgs::msg::Float32();
        t_msg.data = static_cast<float>(throttle_cmd);
        RCLCPP_INFO(this->get_logger(), "Throttle Command: %.3f", throttle_cmd);
        throttle_pub_->publish(t_msg);

        // RCLCPP_INFO(this->get_logger(), "Published commands - steer: %.3f, throttle: %.3f", steer_cmd, throttle_cmd);

        // Update previous commands for next iteration
        prev_steer_ = steer_cmd;
        prev_throttle_ = throttle_cmd;
    }

private:
    // Member variables
    message_filters::Subscriber<geometry_msgs::msg::Point> ips_sub_;
    message_filters::Subscriber<std_msgs::msg::Float32> speed_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::msg::Point, std_msgs::msg::Float32>> sync_;
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;

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
    double param_[14]; // Parameter array for GRAMPC vehicle model (matches main_VEHICLE.c)

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
