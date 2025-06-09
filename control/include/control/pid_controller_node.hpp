#ifndef SECOND_CPP_PKG__PID_CONTROLLER_NODE_HPP_
#define SECOND_CPP_PKG__PID_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace second_cpp_pkg
{

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode();

private:
    void compute_pid(double current_value);

    // PID parameters
    double kp_;
    double ki_;
    double kd_;

    // PID state
    double setpoint_;
    double integral_;
    double prev_error_;
    rclcpp::Time last_time_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr feedback_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;
};

}  // namespace second_cpp_pkg

#endif  // SECOND_CPP_PKG__PID_CONTROLLER_NODE_HPP_
