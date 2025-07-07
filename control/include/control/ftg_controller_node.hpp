#ifndef CONTROL__FOLLOW_GAP_NODE_HPP_
#define CONTROL__FOLLOW_GAP_NODE_HPP_

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "control/common.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

namespace ftg_controller
{

    class FTGController : public rclcpp::Node
    {
    public:
        FTGController();

    private:
        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        std::vector<float> preprocess_lidar(const std::vector<float> &ranges);
        std::pair<int, int> find_max_gap(const std::vector<float> &ranges);
        int find_best_point(int start_idx, int end_idx);
        void drive_towards_point(int best_point_idx, float angle_min, float angle_increment);

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;

        // Parameters
        float disparity_threshold_;
        int bubble_radius_;
        float forward_speed_;
    };

}

#endif
