#include "geometry_msgs/msg/point.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

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

        // Initialize GRAMPC with safety checks
        grampc_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "About to initialize GRAMPC...");
        grampc_init(&grampc_, (void*)&ctx_);
        RCLCPP_INFO(this->get_logger(), "GRAMPC init completed");
        
        if (!grampc_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GRAMPC solver");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "GRAMPC pointer is valid");
        
        if (!grampc_->param || !grampc_->opt || !grampc_->sol) {
            RCLCPP_ERROR(this->get_logger(), "GRAMPC structures not properly allocated");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "GRAMPC structures are valid");

        // Set problem dimensions - these MUST match exactly with ocp_dim() in mpcc_model.c
        grampc_->param->Nx = 4;  // [x, y, theta, v] - basic vehicle state
        grampc_->param->Nu = 2;  // [v_cmd, steering_angle]
        grampc_->param->Ng = 1;  // 1 inequality constraint (velocity limit)
        grampc_->param->Nh = 0;
        grampc_->param->Np = 0;
        grampc_->param->NhT = 0;
        grampc_->param->NgT = 0;

        // Basic MPC parameters - shorter horizon for responsiveness
        grampc_->param->Thor = 0.8;     // Horizon time (0.8 seconds for basic MPC)
        grampc_->param->dt = 0.08;      // Time step for discretization

        // Solver options for basic MPC - more robust settings
        grampc_->opt->Nhor = 10;        // Number of discretization steps (Thor/dt = 0.8/0.08 = 10)
        grampc_->opt->MaxGradIter = 20; // More iterations for better convergence
        grampc_->opt->MaxMultIter = 5;  // More multiplier iterations
        grampc_->opt->Integrator = 0;   // Euler
        grampc_->opt->LineSearchType = 0;  // Disable line search to avoid status 8
        grampc_->opt->TimeDiscretization = 0;  // Equidistant
        grampc_->opt->LineSearchMax = 50;      // More line search iterations if enabled
        
        // CRITICAL: Set initial state and control before running
        std::vector<double> x0_init = {0.0, 0.0, 0.0, 0.0};  // 4D state [x, y, theta, v]
        std::vector<double> u0_init = {0.0, 0.0};
        
        // Safety check before setting parameters
        if (grampc_ && grampc_->param) {
            grampc_setparam_real_vector(grampc_, "x0", x0_init.data());
            grampc_setparam_real_vector(grampc_, "u0", u0_init.data());
            
            // Set initial reference (required for first solve)
            std::vector<double> xref_init = {0.0, 0.0, 0.0, 0.0};  // 4D reference [x, y, theta, v]
            grampc_setparam_real_vector(grampc_, "xdes", xref_init.data());

            // F1TENTH-style control bounds - [v_cmd, steering_angle]
            double umin[2] = {ctx_.a_min, -ctx_.delta_max};
            double umax[2] = {ctx_.a_max, ctx_.delta_max};
            grampc_setparam_real_vector(grampc_, "umin", umin);
            grampc_setparam_real_vector(grampc_, "umax", umax);
            
            // Set constraint tolerance (following Vehicle example)
            double ConstraintsAbsTol[1] = {1e-2}; // For 1 constraint
            grampc_setopt_real_vector(grampc_, "ConstraintsAbsTol", ConstraintsAbsTol);
            
            // Set manual penalty values since we disabled penalty estimation
            double pen[1] = {1.0}; // Manual penalty for velocity constraint
            grampc_setopt_real_vector(grampc_, "PenaltyMin", pen);
            
            RCLCPP_INFO(this->get_logger(), "GRAMPC parameters set successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cannot set GRAMPC parameters - structures not initialized");
        }
        
        // Estimate penalty parameters AFTER setting all parameters (following Vehicle example)
        // grampc_estim_penmin(grampc_, 1);  // Temporarily disable to isolate segfault
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
        // Safety check: Ensure GRAMPC is properly initialized
        if (!grampc_ || !grampc_->param || !grampc_->sol) {
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
        if (!has_prev_fix_) {
            RCLCPP_DEBUG(this->get_logger(), "No IPS data received yet, skipping control loop");
            return;
        }

        // Update progress state based on vehicle's position along the path
        // Find the closest point on the path to current vehicle position
        Eigen::Vector2d vehicle_pos(x_, y_);
        double min_dist = std::numeric_limits<double>::max();
        double best_s = s_;
        
        // Sample along the path to find the closest point
        for (double test_s = 0.0; test_s <= path_->total_length(); test_s += 0.2) {
            Eigen::Vector2d path_point = path_->position(test_s);
            double dist = (vehicle_pos - path_point).norm();
            if (dist < min_dist) {
                min_dist = dist;
                best_s = test_s;
            }
        }
        s_ = best_s;

        // Get reference point ahead on the path for path following
        double ref_distance = 1.0; // Look-ahead distance in meters
        double s_ref = std::min(s_ + ref_distance, path_->total_length());
        
        Eigen::Vector2d ref_pos = path_->position(s_ref);
        double ref_yaw = path_->yaw(s_ref);
        double ref_speed = 0.5; // Target speed for path following

        // Current vehicle state
        std::vector<double> current_state = {x_, y_, yaw_, v_};
        
        // Desired target state (path following)
        std::vector<double> target_state = {ref_pos.x(), ref_pos.y(), ref_yaw, ref_speed};

        // Set current state as initial condition
        grampc_setparam_real_vector(grampc_, "x0", current_state.data());
        grampc_setparam_real_vector(grampc_, "xdes", target_state.data());

        double steer_cmd = 0.0;
        double throttle_cmd = 0.0;

        // Enhanced GRAMPC solver execution with debugging
        auto solver_start = std::chrono::high_resolution_clock::now();
        
        // Extensive validation before calling grampc_run
        if (!grampc_->sol->xnext || !grampc_->sol->unext) {
            RCLCPP_ERROR(this->get_logger(), "GRAMPC solution vectors not allocated");
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
        } else if (!grampc_->rws->x || !grampc_->rws->u) {
            RCLCPP_ERROR(this->get_logger(), "GRAMPC workspace vectors not allocated");
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
        } else {
            // Print current state for debugging
            RCLCPP_INFO(this->get_logger(), "About to call grampc_run with state: [%.3f, %.3f, %.3f, %.3f]", 
                grampc_->param->x0[0], grampc_->param->x0[1], grampc_->param->x0[2], grampc_->param->x0[3]);
            
            try {
                // Try to run GRAMPC with exception handling
                RCLCPP_DEBUG(this->get_logger(), "Calling grampc_run...");
                grampc_run(grampc_);
                RCLCPP_DEBUG(this->get_logger(), "grampc_run completed");
                
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Exception during grampc_run");
                steer_cmd = prev_steer_;
                throttle_cmd = prev_throttle_;
                return;
            }
        }
        
        if (grampc_->sol->status > 0) {
            RCLCPP_WARN(this->get_logger(), "GRAMPC solver failed with status %d", grampc_->sol->status);
            steer_cmd = prev_steer_;
            throttle_cmd = prev_throttle_;
        } else {
            // Extract control from solution
            throttle_cmd = grampc_->sol->unext[0];  // v_cmd
            steer_cmd = grampc_->sol->unext[1];     // steering angle

            RCLCPP_INFO(this->get_logger(), "MPCC Solution - v_cmd: %.3f, steer: %.3f",
                        throttle_cmd, steer_cmd);
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

        RCLCPP_INFO(this->get_logger(), "Published commands - steer: %.3f, throttle: %.3f",
                    steer_cmd, throttle_cmd);

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
    
    // Vehicle state
    double x_, y_, yaw_, v_, s_;
    double last_x_ = 0.0, last_y_ = 0.0, last_time_ = 0.0;
    bool has_prev_fix_ = false;
    
    // Control history for smoothing
    double prev_steer_, prev_throttle_;
    
    // GRAMPC solver and context
    TYPE_GRAMPC_POINTER(grampc_);
    mpcc_ctx_t ctx_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCCGrampcNode>());
    rclcpp::shutdown();
    return 0;
}
