#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP
#pragma once

#include "control/tools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <limits>
#include "control/common.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <deque>
#include <cmath>
#include <optional>
#include <algorithm>
#include <numeric>
#include "autodrive_msgs/msg/vehiclestate.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController();

private:
    // ROS handles
    rclcpp::Subscription<autodrive_msgs::msg::Vehiclestate>::SharedPtr vehicle_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_, throttle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_, cp_pub_, est_pos_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr race_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void vehicle_callback(const autodrive_msgs::msg::Vehiclestate::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void main_control_loop();
    void publish_estimated_position();
    void load_raceline_csv(const std::string &file_path);
    double get_yaw_from_imu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
    void update_lookahead_distance(double speed);
    std::pair<Eigen::Vector2d, std::optional<Eigen::Vector2d>> find_lookahead_point();
    double calculate_alpha(const Eigen::Vector2d &goal_point, double yaw);
    double calculate_heading_angle(double alpha);
    double calculate_curvature(double alpha);
    double calculate_deviation(const Eigen::Vector2d &pos, const Eigen::Vector2d &closest);
    double calculate_max_velocity_pure_pursuit(double curvature);
    double calculate_min_deviation_pure_pursuit(double area);
    double adjust_beta(double current_speed, double area);
    double convex_combination(double max_v_pp, double min_d_pp, double cur_spd, double area);
    void publish_markers(const Eigen::Vector2d &closest_point, const Eigen::Vector2d &goal_point);
    void publish_raceline_visualization();
    void publish_control_commands();

    // State & Params
    std::vector<Eigen::Vector2d> path_;
    std::vector<double> velocities_;
    std::optional<Eigen::Vector2d> previous_position_;
    std::optional<Eigen::Vector2d> current_position_;
    std::array<double, 4> current_quaternion_;
    std::deque<double> area_window_;
    double max_speed_, min_speed_, max_lookahead_, min_lookahead_, wheelbase_;
    double lookahead_distance_ = 1.0, beta_, previous_deviation_ = 0.0, total_area_ = 0.0, control_velocity_ = 0.1, heading_angle_ = 0.5;
    double heading_scale_, area_threshold_, speed_factor_;
    double r_ = 0.8;
    size_t window_size_, vel_window;
    double current_speed_;
    double yaw_;

    // **PID Controller for throttle**
    PIDController throttle_pid_;
    double pid_kp_, pid_ki_, pid_kd_;
    rclcpp::Time last_control_time_;
};

#endif // PURE_PURSUIT_CONTROLLER_HPP
