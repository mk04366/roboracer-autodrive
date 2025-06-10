#ifndef SECOND_CPP_PKG__PID_CONTROLLER_NODE_HPP_
#define SECOND_CPP_PKG__PID_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace pid_controller_node
{

    class PIDControllerNode : public rclcpp::Node
    {
    public:
        PIDControllerNode();

    private:
        void compute_pid_steering(double current_value);
        void compute_pid_throttle(double current_value);

        // PID parameters
        double kiThrottle;
        double kdThrottle;
        double kpThrottle;

        double kiSteering;
        double kdSteering;
        double kpSteering;

        // PID states
        double setpointSteering;
        double integralSteering;
        double prevErrorSteering;
        double setpointThrottle;
        double integralThrottle;
        double prevErrorThrottle;

        rclcpp::Time lastTime;

        // ROS interfaces
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr targetThrottleSub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr targetSteeringSub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr feedbackThrottleSub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr feedbackSteeringSub;

        //final control commands given to the simulator
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_command_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_command_pub;
    };

}

#endif
