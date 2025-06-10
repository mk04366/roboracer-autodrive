#include "pid_controller_node.hpp"

namespace pid_controller_node
{

    PIDControllerNode::PIDControllerNode()
        : Node("pid_controller_node"),
          kpSteeringAngle(1.0), kpThrottle(1.0), kiThrottle(0.0), kiSteeringAngle(0.0), kdThrottle(0.1), kdSteeringAngle(0.1),
          setpointThrottle(0.0), setpointSteeringAngle(0.0), integralThrottle(0.0), integralSteeringAngle(0.0),
          prevErrorThrottle(0.0), prevErrorSteeringAngle(0.0)
    {
        declare_parameter("kp", 1.0);
        declare_parameter("ki", 0.0);
        declare_parameter("kd", 0.1);

        get_parameter("kp", kp_);
        get_parameter("ki", ki_);
        get_parameter("kd", kd_);

        targetThrottleSub = this->create_subscription<std_msgs::msg::Float64>(
            "throttle_command", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg)
            {
                setpoint_ = msg->data;
            });

        feedbackThrottleSub = this->create_subscription<std_msgs::msg::Float64>(
            "throttle", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg)
            {
                compute_pid(msg->data);
            });

        control_pub_ = this->create_publisher<std_msgs::msg::Float64>("control", 10);

        last_time_ = this->now();
    }

    void PIDControllerNode::compute_pid(double current_value)
    {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double error = setpoint_ - current_value;
        integral_ += error * dt;
        double derivative = (dt > 0.0) ? (error - prev_error_) / dt : 0.0;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        prev_error_ = error;

        auto msg = std_msgs::msg::Float64();
        msg.data = output;
        control_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Setpoint: %.2f | Feedback: %.2f | Output: %.2f",
                    setpoint_, current_value, output);
    }

}
