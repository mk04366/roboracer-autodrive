#ifndef CONTROL_GRAMPC__MPCC_GRAMPC_NODE_HPP_
#define CONTROL_GRAMPC__MPCC_GRAMPC_NODE_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "control_grampc/path_utils.hpp"
#include "control_grampc/mpcc_model.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "autodrive_msgs/msg/vehiclestate.hpp"

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
#define NH 0

class MPCCGrampcNode : public rclcpp::Node
{
public:
    MPCCGrampcNode();
    ~MPCCGrampcNode();

private:
    void initializePathPosition();
    void initGrampcParams();
    void controlLoop();
    void publishPath();
    void publishTarget(const Eigen::Vector2d &point, double heading);
    double getYawFromImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
    void vehicleCallback(const autodrive_msgs::msg::Vehiclestate::SharedPtr msg);
    // ROS publishers/subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr path_timer_;
    rclcpp::Subscription<autodrive_msgs::msg::Vehiclestate>::SharedPtr vehicle_sub_;

    std::shared_ptr<mpcc::Path> path_;
    // Vehicle state - 5D [x, y, theta, kappa, v]
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, v_ = 0.0, kappa_ = 0.0;

    // Path following state
    size_t current_path_idx_ = 0;
    double current_s_ = 0.0;

    // Control history
    double prev_steer_, prev_throttle_;

    // GRAMPC solver and parameter array
    TYPE_GRAMPC_POINTER(grampc_);
    double param_[14];
    double dt_ = 0.0;

    // Vehicle parameters
    double L_;
    double last_yaw_ = 0.0;
    bool first_pose_ = true;
    geometry_msgs::msg::Point last_pos_;
};

#endif // CONTROL_GRAMPC__MPCC_GRAMPC_NODE_HPP_
