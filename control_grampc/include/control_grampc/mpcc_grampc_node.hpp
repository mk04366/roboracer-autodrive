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
#define NH 0


class MPCCGrampcNode : public rclcpp::Node
{
public:
    MPCCGrampcNode();

private:
    void initializePathPosition();
    void initGrampcParams();
    void controlLoop(
        const geometry_msgs::msg::Point::ConstSharedPtr &ips_msg,
        const std_msgs::msg::Float32::ConstSharedPtr &speed_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
    double getYawFromImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
    double computeCurvature(
        const geometry_msgs::msg::Point::ConstSharedPtr &ips_msg,
        double current_yaw);

    // Member variables
    // Subscribers
    message_filters::Subscriber<geometry_msgs::msg::Point> ips_sub_;
    message_filters::Subscriber<std_msgs::msg::Float32> speed_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;

    // Synchronizer
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            geometry_msgs::msg::Point,
            std_msgs::msg::Float32,
            sensor_msgs::msg::Imu>>>
        sync_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;

    std::shared_ptr<mpcc::Path> path_;

    // Vehicle state - 5D [x, y, theta, kappa, v]
    double x_, y_, yaw_, v_;
    double kappa_ = 0.0;          // current curvature
    double steering_angle_ = 0.0; // current steering angle for curvature calculation
    double last_time_ = 0.0;      // for dt calculation

    // Path following state
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
    double last_yaw_ = 0.0;
    bool first_pose_ = true;
    geometry_msgs::msg::Point last_pos_;
    typeRNum rwsReferenceIntegration[2 * NX];
};

#endif // CONTROL_GRAMPC__MPCC_GRAMPC_NODE_HPP_