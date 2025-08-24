#include "control_grampc/mpc_node.h"
#include <cmath>
#include <algorithm>

namespace control_grampc {

MPCNode::MPCNode() 
    : Node("mpc_node")
    , x_(0.0), y_(0.0), theta_(0.0), v_(0.0), s_(0.0)
    , last_x_(0.0), last_y_(0.0), last_time_(0.0)
    , has_prev_fix_(false)
    , input_index_(0)
    , first_scan_received_(false)
    , control_counter_(0) {
    
    // Declare parameters
    path_file_ = this->declare_parameter<std::string>("path_file", 
        "/home/ammar/ros2_ws/src/global-planning/outputs/map5/traj_race_cl.csv");
    control_frequency_ = this->declare_parameter<double>("control_frequency", 20.0);
    lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 2.0);
    target_velocity_ = this->declare_parameter<double>("target_velocity", 2.0);
    double ref_lookahead = this->declare_parameter<double>("mpc.ref_lookahead_sec", 0.5);
    
    // Load path
    try {
        path_ = std::make_shared<mpcc::Path>(mpcc::load_path_from_csv(path_file_));
        if (path_->total_length() < 1e-3) {
            RCLCPP_ERROR(this->get_logger(), "Path is too short or invalid");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded path with length: %.2f m", path_->total_length());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load path: %s", e.what());
        return;
    }
    
    // Initialize MPC without node pointer first
    mpc_ = std::make_unique<MPC>();
    mpc_->setRefLookahead(ref_lookahead);
    
    // Set up ROS2 interfaces
    ips_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/autodrive/f1tenth_1/ips", 10,
        std::bind(&MPCNode::ipsCallback, this, std::placeholders::_1));
        
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/autodrive/f1tenth_1/lidar", 10,
        std::bind(&MPCNode::scanCallback, this, std::placeholders::_1));
    
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/autodrive/f1tenth_1/throttle_command", 10);
        
    steering_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/autodrive/f1tenth_1/steering_command", 10);
        
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "mpc_visualization", 10);
    
    // Control timer
    auto control_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
    control_timer_ = this->create_wall_timer(control_period, 
        std::bind(&MPCNode::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "MPC Node initialized");
}

void MPCNode::initializeMPC() {
    // Set the node pointer and initialize GRAMPC
    if (mpc_) {
        mpc_->setNode(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "MPC controller initialized successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPC controller");
    }
}

void MPCNode::ipsCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    const double now_sec = this->now().seconds();
    const double dx = msg->x - last_x_;
    const double dy = msg->y - last_y_;
    const double dt = now_sec - last_time_;
    
    x_ = msg->x;
    y_ = msg->y;
    
    if (has_prev_fix_ && dt > 1e-3) {
        // Estimate heading from motion
        const double new_theta = std::atan2(dy, dx);
        if (std::abs(dx) > 0.01 || std::abs(dy) > 0.01) {
            // Update heading with simple smoothing
            const double dtheta = std::atan2(std::sin(new_theta - theta_), 
                                           std::cos(new_theta - theta_));
            theta_ += 0.3 * dtheta;
        }
        
        // Estimate velocity
        const double speed = std::sqrt(dx * dx + dy * dy) / dt;
        v_ = 0.8 * v_ + 0.2 * speed;  // Low-pass filter
    }
    
    has_prev_fix_ = true;
    
    last_x_ = x_;
    last_y_ = y_;
    last_time_ = now_sec;
    
    // Update progress along path
    updateProgress();
    
    // Update vehicle state
    updateVehicleState();
}

void MPCNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    
    if (mpc_) {
        mpc_->updateScan(msg);
    }
    
    if (!first_scan_received_) {
        first_scan_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First scan received");
    }
}

void MPCNode::updateVehicleState() {
    current_state_ = State(x_, y_, theta_, v_);
    
    // Log state for debugging
    static int log_counter = 0;
    if (++log_counter % 50 == 0) {  // Log every 50 updates
        RCLCPP_INFO(this->get_logger(), "Vehicle state: [%.3f, %.3f, %.3f, %.3f]", 
                   x_, y_, theta_, v_);
    }
}

void MPCNode::updateProgress() {
    if (!path_ || path_->total_length() < 1e-3) {
        return;
    }
    
    // Find closest point on path
    Eigen::Vector2d vehicle_pos(x_, y_);
    double min_dist = std::numeric_limits<double>::max();
    double best_s = s_;
    
    // Search around current progress
    const double search_range = std::max(2.0, v_ * 0.5);  // Adaptive search range
    const double s_start = std::max(0.0, s_ - search_range);
    const double s_end = std::min(path_->total_length(), s_ + search_range);
    
    for (double test_s = s_start; test_s <= s_end; test_s += 0.1) {
        Eigen::Vector2d test_pos = path_->interpolate(test_s);
        double dist = (vehicle_pos - test_pos).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_s = test_s;
        }
    }
    
    // Update progress (ensure forward motion)
    s_ = std::max(s_, best_s);
    
    // Handle path completion
    if (s_ >= path_->total_length() - 0.5) {
        s_ = 0.0;  // Loop back to start
        RCLCPP_INFO(this->get_logger(), "Path completed, looping back");
    }
}

void MPCNode::generateReferenceTrajectory() {
    reference_trajectory_.clear();
    
    if (!path_ || !mpc_) {
        return;
    }
    
    const int horizon = mpc_->horizon();
    const double dt = mpc_->dt();
    
    // Debug information
    if (control_counter_ % 20 == 0) {  // Log every second at 20Hz
        RCLCPP_INFO(this->get_logger(), "Path following: vehicle=[%.3f, %.3f], s_=%.3f, path_length=%.3f", 
                   x_, y_, s_, path_->total_length());
    }
    
    for (int i = 0; i < horizon; ++i) {
        // Project future position along path
        const double future_s = s_ + i * dt * target_velocity_;
        const double clamped_s = std::min(future_s, path_->total_length());
        
        // Get reference position
        Eigen::Vector2d ref_pos = path_->interpolate(clamped_s);
        
        // Get reference heading (tangent to path)
        double ref_theta = theta_;  // Default to current heading
        if (clamped_s < path_->total_length() - 0.1) {
            Eigen::Vector2d forward_pos = path_->interpolate(clamped_s + 0.1);
            Eigen::Vector2d tangent = forward_pos - ref_pos;
            if (tangent.norm() > 1e-6) {
                ref_theta = std::atan2(tangent.y(), tangent.x());
            }
        }
        
        reference_trajectory_.emplace_back(ref_pos.x(), ref_pos.y(), ref_theta, target_velocity_);
        
        // Debug first reference point
        if (i == 0 && control_counter_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "First ref point: [%.3f, %.3f] at s=%.3f", 
                       ref_pos.x(), ref_pos.y(), clamped_s);
        }
    }
}

void MPCNode::controlLoop() {
    control_counter_++;
    
    if (!has_prev_fix_ || !first_scan_received_ || !path_ || !mpc_) {
        // Publish safe defaults
        publishControl(Input(0.0, 0.0));
        if (control_counter_ % 100 == 0) {  // Log every 100 calls
            RCLCPP_WARN(this->get_logger(), "Control loop waiting for initialization: fix=%d, scan=%d, path=%d, mpc=%d", 
                       has_prev_fix_, first_scan_received_, path_ != nullptr, mpc_ != nullptr);
        }
        return;
    }
    
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    
    // Generate reference trajectory
    generateReferenceTrajectory();
    
    if (reference_trajectory_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty reference trajectory");
        publishControl(Input(0.0, 0.0));
        return;
    }
    
    if (control_counter_ % 100 == 0) {  // Log every 100 calls
        RCLCPP_INFO(this->get_logger(), "Control loop running, ref_traj_size=%zu", reference_trajectory_.size());
    }
    
    // Update MPC
    mpc_->update(current_state_, current_input_, reference_trajectory_);
    
    // Get control input
    Input control_input = getNextInput();
    
    // Publish control
    publishControl(control_input);
    
    // Store current input
    current_input_ = control_input;
    
    // Visualize
    mpc_->visualize();
    
    last_control_time_ = std::chrono::steady_clock::now();
}

Input MPCNode::getNextInput() {
    if (!mpc_ || !mpc_->hasSolution()) {
        // Return safe default
        return Input(std::min(1.0, target_velocity_), 0.0);
    }
    
    const auto& trajectory = mpc_->getSolvedTrajectory();
    
    if (input_index_ >= trajectory.size()) {
        RCLCPP_WARN(this->get_logger(), "Input index out of bounds, using first input");
        input_index_ = 0;
    }
    
    if (trajectory.empty()) {
        return Input(std::min(1.0, target_velocity_), 0.0);
    }
    
    Input input = trajectory[0];  // Always use first input (receding horizon)
    
    // Apply safety limits
    const double max_steering = 0.4;
    const double max_velocity = 5.0;
    
    input.setSteeringAngle(std::clamp(input.steeringAngle(), -max_steering, max_steering));
    input.setVelocity(std::clamp(input.velocity(), 0.0, max_velocity));
    
    return input;
}

void MPCNode::publishControl(const Input& input) {
    auto throttle_msg = std_msgs::msg::Float32();
    auto steering_msg = std_msgs::msg::Float32();
    
    // Convert velocity to throttle (simple mapping)
    throttle_msg.data = static_cast<float>(input.velocity() / 5.0);
    // Scale steering to normalized range expected downstream (assuming delta_max=0.4 rad)
    const double delta_max = 0.4;
    double steer_norm = std::clamp(input.steeringAngle() / delta_max, -1.0, 1.0);
    steering_msg.data = static_cast<float>(steer_norm);
    
    throttle_pub_->publish(throttle_msg);
    steering_pub_->publish(steering_msg);
    
    // Log steering values for debugging
    static int log_counter = 0;
    if (++log_counter % 20 == 0) {  // Log every 20 calls
        RCLCPP_INFO(this->get_logger(), "Control: throttle=%.3f, steer_norm=%.3f, steer_rad=%.3f", 
                     throttle_msg.data, steering_msg.data, input.steeringAngle());
    }
}

} // namespace control_grampc
