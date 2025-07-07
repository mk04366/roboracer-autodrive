#ifndef CONTROL__PID_CONTROLLER_NODE_HPP_
#define CONTROL__PID_CONTROLLER_NODE_HPP_

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "control/common.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>

namespace pid_controller_node
{
    /**
     * @brief PID Controller Node for controlling steering and throttle.
     *
     * This node subscribes to target commands for steering and throttle,
     * computes the PID control outputs, and publishes the control commands.
     */

    class PIDControllerNode : public rclcpp::Node
    {
    public:
        PIDControllerNode();

    private:
        void compute_steering_control();
        void compute_throttle_control();
        void process_lidar_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        // PID parameters
        double kp_steering = 1.0;
        double ki_steering = 0.1;
        double kd_steering = 0.2;

        double kp_throttle = 0.2;
        double ki_throttle = 1e-3;
        double kd_throttle = 0.1;

        double wall_following_error = 0.0;

        // PID states
        double integralSteering = 0.0;
        double prevErrorSteering = 0.0;
        double integralThrottle = 0.0;
        double prevErrorThrottle = 0.0;

        // Control parameters
        double setpointSpeed = 0.5;         // Desired speed in m/s
        double desired_center_offset = 0.0; // Desired offset from the wall

        // low pass filter parameters
        double smoothed_throttle_output = 0.0;
        double alpha = 0.1;

        // Feedback states
        double feedbackSpeed;
        double feedbackSteering;
        double feedbackThrottle;
        rclcpp::TimerBase::SharedPtr control_timer;

        rclcpp::Time lastTimeSteering;
        rclcpp::Time lastTimeThrottle;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedbackSpeedSub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr feedbackLidarSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedbackSteeringSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedbackThrottleSub;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_command_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_command_pub;
    };

}

#endif
