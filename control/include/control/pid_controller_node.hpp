#ifndef CONTROL__PID_CONTROLLER_NODE_HPP_
#define CONTROL__PID_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace pid_controller_node
{

    template <typename T>
    T clamp(const T &v, const T &lo, const T &hi)
    {
        return (v < lo) ? lo : (v > hi) ? hi
                                        : v;
    }
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
        double ki_steering = 1e-3;
        double kd_steering = 1e-2;
        double kp_steering = 1.0;
        double kp_throttle = 1.0;
        double ki_throttle = 0.1;
        double kd_throttle = 0.05;

        double wall_following_error = 0.0;

        // PID states
        double setpoint = 1.0; // Desired distance from wall in meters
        double integral = 0.0;
        double prevError = 0.0;
        double setpointSpeed = 1.0; // Desired speed in m/s
        double integralThrottle = 0.0;
        double prevErrorThrottle = 0.0;

        // Feedback states
        double feedbackSpeed;
        double feedbackLidar;
        double feedbackSteering;
        double feedbackThrottle;
        rclcpp::TimerBase::SharedPtr control_timer;

        rclcpp::Time lastTimeSteering;
        rclcpp::Time lastTimeThrottle;

        // ROS interfaces
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr targetSub;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedbackSpeedSub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr feedbackLidarSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedbackSteeringSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedbackThrottleSub;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_command_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_command_pub;
    };

}

#endif
