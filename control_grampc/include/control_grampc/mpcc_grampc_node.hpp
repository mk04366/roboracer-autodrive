#ifndef CONTROL_GRAMPC__MPCC_GRAMPC_NODE_HPP_
#define CONTROL_GRAMPC__MPCC_GRAMPC_NODE_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "control_grampc/path_utils.hpp"
#include "control_grampc/mpcc_model.h"

extern "C"
{
#include "grampc.h"
#include "grampc_mess.h"
}

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>

#define NX 5
#define NU 2
#define NH 3

class MPCCGrampcNode : public rclcpp::Node
{
public:
    MPCCGrampcNode();

private:
    void ipsCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void speedCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void controlTimerCallback();
    void initializePathPosition();
    void controlLoop();

    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ips_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Data availability and timing
    bool has_ips_data_ = false;
    bool has_speed_data_ = false;
    bool has_imu_data_ = false;
    rclcpp::Time last_ips_time_;
    rclcpp::Time last_speed_time_;
    rclcpp::Time last_imu_time_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;

    std::shared_ptr<mpcc::Path> path_;

    // Vehicle state - 5D [x, y, theta, kappa, v]
    double x_, y_, yaw_, v_;
    double kappa_ = 0.0;          // current curvature
    double steering_angle_ = 0.0; // current steering angle for curvature calculation
    double last_time_ = 0.0;      // for dt calculation

    // Path following state
    bool path_initialized_ = false;
    size_t current_path_idx_ = 0;
    double current_s_ = 0.0; // Current arc length position

    // Control history for smoothing
    double prev_steer_, prev_throttle_;

    // GRAMPC solver and parameter array
    TYPE_GRAMPC_POINTER(grampc_);
    double param_[14]; // Parameter array for GRAMPC vehicle model (matches main_VEHICLE.c)
    double dt_ = 0.0;
    double t = 0.0; // Current time for GRAMPC integration

    // Easy access to key parameters
    double L_, delta_max_, v_max_;
};

#endif  // CONTROL_GRAMPC__MPCC_GRAMPC_NODE_HPP_