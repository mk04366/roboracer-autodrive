#include "geometry_msgs/msg/point.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "control_grampc/path_utils.hpp"
#include "control_grampc/mpcc_model.h"
extern "C" {
#include "grampc.h"
}
#include "rclcpp/rclcpp.hpp"

#include "control_grampc/path_utils.hpp"
#include "control_grampc/mpcc_model.h"
extern "C" {
#include "grampc.h"
}
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath> // For M_PI and std::abs
#include <limits> // For std::numeric_limits

using std::placeholders::_1;

// Helper function to wrap angle to [-pi, pi]
double wrap_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

class MPCCGrampcNode : public rclcpp::Node
{
public:
    MPCCGrampcNode()
        : Node("mpcc_grampc_node"), x_(0.0), y_(0.0), yaw_(0.0), v_(0.0), s_(0.0),
          prev_steer_(0.0), prev_throttle_(0.0)
    {
        // Load path from CSV file
        std::string csv_file = this->declare_parameter<std::string>("path_csv", "/home/ammar/ros2_ws/src/global-planning/outputs/map5/traj_race_cl.csv");
        path_ = std::make_shared<mpcc::Path>(mpcc::load_path_from_csv(csv_file));
        
        if (path_ && path_->total_length() > 1e-3) {
            RCLCPP_INFO(this->get_logger(), "Path loaded successfully: %s, length: %f", csv_file.c_str(), path_->total_length());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path from: %s", csv_file.c_str());
        }

        // ROS interfaces
        ips_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/autodrive/f1tenth_1/ips", 30, std::bind(&MPCCGrampcNode::ipsCallback, this, _1));

        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MPCCGrampcNode::controlLoop, this));

        // GRAMPC setup (v2.2 API) with user context
        ctx_.L = 0.33;
        ctx_.delta_max = 0.4;

        ctx_.a_min = 0;         
        ctx_.a_max = 0.1;

        ctx_.v_max = 1.0;

        ctx_.a_lat_max = 1.2;      // lateral accel cap

        // Track boundary settings (in meters from centerline)
        ctx_.track_width_left = 1.5;
        ctx_.track_width_right = 1.5;

        // Basic MPC weights - simplified
        ctx_.w_ey = 10.0;          // not used in basic MPC  
        ctx_.w_epsi = 5.0;         // not used in basic MPC
        ctx_.w_v = 1.0;            // velocity tracking weight
        ctx_.w_s = 0.0;            // not used in basic MPC
        ctx_.w_u = 0.1;            // control effort weight
        ctx_.w_du = 0.0;           // not used in basic MPC
        ctx_.w_term = 5.0;         // terminal weight
        ctx_.w_barrier = 0.0;      // not used in basic MPC
        ctx_.u_prev[0] = 0.0; ctx_.u_prev[1] = 0.0;

        // Initialize GRAMPC
        grampc_ = nullptr;
        grampc_init(&grampc_, (void*)&ctx_);

        // Set problem dimensions - these MUST be set before calling grampc_setparam
        grampc_->param->Nx = 5;  // [x, y, psi, v, s] - extended MPCC state
        grampc_->param->Nu = 2;  // [steer, throttle]
        grampc_->param->Ng = 0;
        grampc_->param->Nh = 0;
        grampc_->param->Np = 0;
        grampc_->param->NhT = 0;
        grampc_->param->NgT = 0;

        // Basic MPC parameters - shorter horizon for responsiveness
        grampc_->param->Thor = 0.8;     // Horizon time (0.8 seconds for basic MPC)
        grampc_->param->dt = 0.08;      // Time step for discretization

        // Solver options for basic MPC
        grampc_->opt->Nhor = 10;        // Number of discretization steps (Thor/dt = 0.8/0.08 = 10)
        grampc_->opt->MaxGradIter = 5;  // Fewer iterations for speed
        grampc_->opt->MaxMultIter = 2;  // Fewer multiplier iterations
        grampc_->opt->Integrator = 0;   // Euler
        grampc_->opt->LineSearchType = 1;  // Enable line search for better convergence
        grampc_->opt->TimeDiscretization = 0;  // Equidistant
        
        // CRITICAL: Set initial state and control before running
        std::vector<double> x0_init = {0.0, 0.0, 0.0, 0.0, 0.0};  // 5D state [x, y, psi, v, s]
        std::vector<double> u0_init = {0.0, 0.0};
        grampc_setparam_real_vector(grampc_, "x0", x0_init.data());
        grampc_setparam_real_vector(grampc_, "u0", u0_init.data());
        
        // Set initial reference (required for first solve)
        std::vector<double> xref_init = {0.0, 0.0, 0.0, 0.0, 0.0};  // 5D reference [ey, epsi, v, s, unused]
        grampc_setparam_real_vector(grampc_, "xdes", xref_init.data());

        // F1TENTH-style control bounds - consistent with ctx_ values
        double umin[2] = {-ctx_.delta_max, ctx_.a_min};
        double umax[2] = {ctx_.delta_max, ctx_.a_max};
        grampc_setparam_real_vector(grampc_, "umin", umin);
        grampc_setparam_real_vector(grampc_, "umax", umax);
        
        // Estimate penalty parameters (following Vehicle example)
        grampc_estim_penmin(grampc_, 1);
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
            yaw_ += 0.5 * dyaw; // smooth update

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
        if (!path_ || path_->total_length() < 1e-3)
        {
            RCLCPP_WARN(this->get_logger(), "No path found, skipping control loop");
            return;
        }

        // Update progress state based on vehicle's position along the path
        // Find the closest point on the path to current vehicle position
        Eigen::Vector2d vehicle_pos(x_, y_);
        double min_dist = std::numeric_limits<double>::max();
        double best_s = s_;
        
        // First time initialization: find closest point globally
        if (s_ < 0.1) {
            for (double test_s = 0.0; test_s <= path_->total_length(); test_s += 0.2) {
                Eigen::Vector2d test_pos = path_->interpolate(test_s);
                double dist = (vehicle_pos - test_pos).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    best_s = test_s;
                }
            }
            s_ = best_s;
        } else {
            // Normal operation: search around current progress
            for (double ds = -0.5; ds <= 1.5; ds += 0.05) {
                double test_s = std::clamp(s_ + ds, 0.0, path_->total_length());
                Eigen::Vector2d test_pos = path_->interpolate(test_s);
                double dist = (vehicle_pos - test_pos).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    best_s = test_s;
                }
            }
            
            // Update progress, ensuring forward motion
            if (best_s >= s_ - 0.1) {  // Allow small backward tolerance
                s_ = best_s;
            } else {
                // Fallback: advance based on velocity
                s_ = std::min(s_ + v_ * 0.05, path_->total_length()); // 0.05s control loop
            }
        }

        // Compute centerline-framed errors
        Eigen::Vector2d path_pos = path_->interpolate(s_);
        double path_heading = path_->heading(s_);
        Eigen::Vector2d path_tangent(cos(path_heading), sin(path_heading));
        Eigen::Vector2d path_normal(-sin(path_heading), cos(path_heading)); // left normal
        
        // Calculate lateral error (positive = left of centerline, negative = right)
        Eigen::Vector2d error_vector = vehicle_pos - path_pos;
        double e_y = error_vector.dot(path_normal);
        
        // Calculate heading error (positive = turning left relative to path)
        double e_psi = wrap_angle(yaw_ - path_heading);
        
        // Progress target for the horizon
        double target_speed = std::min(ctx_.v_max, 
            std::sqrt(ctx_.a_lat_max / std::max(std::abs(path_->curvature(s_)), 0.1)));
        double s_target = std::min(s_ + target_speed * grampc_->param->Thor, path_->total_length());

        // Debug information
        RCLCPP_INFO(this->get_logger(), 
            "MPCC State - s: %.3f, e_y: %.3f, e_psi: %.3f, v: %.3f, target_v: %.3f", 
            s_, e_y, e_psi, v_, target_speed);

        // Prepare 5D state vector: [x, y, psi, v, s]
        // Note: For the cost function, we'll pass centerline-framed errors via the reference
        std::vector<double> x0 = {x_, y_, yaw_, v_, s_};
        std::vector<double> u0 = {prev_steer_, prev_throttle_};

        grampc_setparam_real_vector(grampc_, "x0", x0.data());
        grampc_setparam_real_vector(grampc_, "u0", u0.data());

        // Set reference as path coordinates for tracking
        // xref = [path_x, path_y, path_psi, v_ref, s_ref]
        std::vector<double> xref = {path_pos.x(), path_pos.y(), path_heading, target_speed, s_target};
        grampc_setparam_real_vector(grampc_, "xdes", xref.data());

        // Run MPC step
        double steer_cmd = 0.0;
        double throttle_cmd = 0.0;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        if (!grampc_ || !grampc_->sol) {
            RCLCPP_ERROR(this->get_logger(), "GRAMPC not properly initialized");
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
        } else {
            // Run the solver
            grampc_run(grampc_);
            
            if (grampc_->sol->status > 0) {
                RCLCPP_WARN(this->get_logger(), "GRAMPC solver failed with status %d", grampc_->sol->status);
                steer_cmd = prev_steer_;
                throttle_cmd = prev_throttle_;
            } else {
                // Extract optimal control
                steer_cmd = grampc_->sol->unext[0];
                throttle_cmd = grampc_->sol->unext[1];
                
                RCLCPP_INFO(this->get_logger(), "MPCC Solution - steer: %.3f, throttle: %.3f", 
                    steer_cmd, throttle_cmd);
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO(this->get_logger(), "GRAMPC solve time: %ld ms", duration.count());

        // Apply control limits and smoothing
        steer_cmd = std::clamp(steer_cmd, -ctx_.delta_max, ctx_.delta_max);
        throttle_cmd = std::clamp(throttle_cmd, ctx_.a_min, ctx_.a_max);
        
        // Smooth control changes (less smoothing for more responsive control)
        double alpha_steer = 0.9;   // Higher for more responsive steering
        double alpha_throttle = 0.8;
        steer_cmd = alpha_steer * steer_cmd + (1.0 - alpha_steer) * prev_steer_;
        throttle_cmd = alpha_throttle * throttle_cmd + (1.0 - alpha_throttle) * prev_throttle_;

        // Update previous control for rate penalty
        ctx_.u_prev[0] = prev_steer_ = steer_cmd;
        ctx_.u_prev[1] = prev_throttle_ = throttle_cmd;

        // Publish control commands
        auto s_msg = std_msgs::msg::Float32();
        s_msg.data = static_cast<float>(steer_cmd);
        steering_pub_->publish(s_msg);

        auto t_msg = std_msgs::msg::Float32();
        t_msg.data = static_cast<float>(throttle_cmd);
        throttle_pub_->publish(t_msg);

        RCLCPP_INFO(this->get_logger(), "Published commands - steer: %.3f, throttle: %.3f", 
            s_msg.data, t_msg.data);
    }

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ips_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    double x_, y_, yaw_, v_, s_;
    double last_x_ = 0.0, last_y_ = 0.0, last_time_ = 0.0;
    bool has_prev_fix_ = false;

    // Path
    std::shared_ptr<mpcc::Path> path_;

    // GRAMPC
    typeGRAMPC *grampc_;

    // User context and previous control for rate penalties
    mpcc_ctx_t ctx_;
    double prev_steer_;
    double prev_throttle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCCGrampcNode>());
    rclcpp::shutdown();
    return 0;
}
