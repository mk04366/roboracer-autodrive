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

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

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

private:
    void initializePathPosition();
    void initGrampcParams();
    void controlLoop();
    void publishPath();
    double getYawFromImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
    double computeCurvature(const geometry_msgs::msg::Point::ConstSharedPtr &ips_msg, double current_yaw);

    void synchronizedCallback(
        const geometry_msgs::msg::Point::ConstSharedPtr &ips_msg,
        const std_msgs::msg::Float32::ConstSharedPtr &speed_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);

    // ROS publishers/subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr path_timer_;

    std::shared_ptr<mpcc::Path> path_;

    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::Point>> ips_sub_;
    std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Float32>> speed_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_sub_;

    using MySyncPolicy = message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::Point,
        std_msgs::msg::Float32,
        sensor_msgs::msg::Imu>;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    // Vehicle state - 5D [x, y, theta, kappa, v]
    double x_, y_, yaw_, v_;
    double kappa_ = 0.0;
    double steering_angle_ = 0.0;
    double last_time_ = 0.0;

    // Path following state
    size_t current_path_idx_ = 0;
    double current_s_ = 0.0;

    // Control history
    double prev_steer_, prev_throttle_;

    // GRAMPC solver and parameter array
    TYPE_GRAMPC_POINTER(grampc_);
    double param_[14];
    double dt_ = 0.0;
    typeRNum t_ = 0.0;

    // Vehicle parameters
    double L_, delta_max_, v_max_;
    double last_yaw_ = 0.0;
    bool first_pose_ = true;
    geometry_msgs::msg::Point last_pos_;
    typeRNum rwsReferenceIntegration[2 * NX];
};

#endif // CONTROL_GRAMPC__MPCC_GRAMPC_NODE_HPP_
