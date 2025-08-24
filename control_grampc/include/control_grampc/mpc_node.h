#ifndef CONTROL_GRAMPC_MPC_NODE_H
#define CONTROL_GRAMPC_MPC_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "control_grampc/mpc.h"
#include "control_grampc/state.h"
#include "control_grampc/input.h"
#include "control_grampc/path_utils.hpp"

#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

namespace control_grampc {

class MPCNode : public rclcpp::Node {
public:
    MPCNode();
    virtual ~MPCNode() = default;

    // Initialize MPC after construction
    void initializeMPC();

private:
    // Callback functions
    void ipsCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void controlLoop();
    
    // Utility functions
    void updateVehicleState();
    void generateReferenceTrajectory();
    Input getNextInput();
    void publishControl(const Input& input);
    void updateProgress();
    
    // Vehicle state
    State current_state_;
    Input current_input_;
    double x_, y_, theta_, v_;
    double s_;  // Progress along path
    
    // Previous state for estimation
    double last_x_, last_y_;
    double last_time_;
    bool has_prev_fix_;
    
    // Path and reference
    std::shared_ptr<mpcc::Path> path_;
    std::vector<State> reference_trajectory_;
    
    // MPC controller
    std::unique_ptr<MPC> mpc_;
    std::vector<Input> solved_trajectory_;
    size_t input_index_;
    
    // Timing and control
    bool first_scan_received_;
    std::chrono::steady_clock::time_point last_control_time_;
    int control_counter_;
    
    // Thread safety
    std::mutex state_mutex_;
    std::mutex scan_mutex_;
    
    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ips_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameters
    double control_frequency_;
    double lookahead_distance_;
    double target_velocity_;
    std::string path_file_;
};

} // namespace control_grampc

#endif // CONTROL_GRAMPC_MPC_NODE_H
