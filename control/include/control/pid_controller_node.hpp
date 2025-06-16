#ifndef SECOND_CPP_PKG__PID_CONTROLLER_NODE_HPP_
#define SECOND_CPP_PKG__PID_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace pid_controller_node
{

    template <typename T>
    T clamp(const T &v, const T &lo, const T &hi)
    {
        return (v < lo) ? lo : (v > hi) ? hi
                                        : v;
    }

    class PIDControllerNode : public rclcpp::Node
    {
    public:
        PIDControllerNode();

    private:
        void compute_pid_steering(double current_value);
        void compute_pid_throttle(double current_throttle);

        // PID parameters
        double kiSteering;
        double kdSteering;
        double kpSteering;

        double kiThrottle;
        double kdThrottle;
        double kpThrottle;

        // PID states
        double setpointSteering;
        double integralSteering;
        double prevErrorSteering;

        double setpointThrottle;
        double integralThrottle;
        double prevErrorThrottle;

        rclcpp::Time lastTimeSteering;
        rclcpp::Time lastTimeThrottle;

        // ROS interfaces
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr targetSteeringSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr targetThrottleSub;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedbackThrottleSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedbackSteeringSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedbackSpeedSub;

        // final control commands given to the simulator
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_command_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_command_pub;
    };

}

#endif
