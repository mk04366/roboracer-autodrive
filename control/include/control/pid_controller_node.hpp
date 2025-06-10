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
        void compute_pid(double current_value);

        // PID parameters
        double kiThrottle, kdThrottle, kpThrottle;
        double kiSteeringAngle, kdSteeringAngle, kpSteeringAngle;

        // PID states
        double setpointSteeringAngle, integralSteeringAngle, prevErrorSteeringAngle;
        double setpointThrottle, integralThrottle ,prevErrorThrottle;
        rclcpp::Time lastTime;

        // ROS interfaces
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr targetThrottleSub, targetSteeringAngleSub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr feedbackThrottleSub, feedbackSteeringAngleSub;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr controlThrottlePub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr controlSteeringAnglePub;
    };

}

#endif
