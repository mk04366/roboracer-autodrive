#include "control/pid_controller_node.hpp"

namespace pid_controller_node
{

    PIDControllerNode::PIDControllerNode()
        : Node("pid_controller_node"),
          kiThrottle(0.0), kdThrottle(0.0), kpThrottle(1.0),
          kiSteering(0.0), kdSteering(0.0), kpSteering(1.0),
          setpointSteering(0.0), integralSteering(0.0), prevErrorSteering(0.0),
          setpointThrottle(0.0), integralThrottle(0.0), prevErrorThrottle(0.0)
    {
        this->declare_parameter("kpThrottle", kpThrottle);
        this->declare_parameter("kiThrottle", kiThrottle);
        this->declare_parameter("kdThrottle", kdThrottle);
        this->declare_parameter("kpSteering", kpSteering);
        this->declare_parameter("kiSteering", kiSteering);
        this->declare_parameter("kdSteering", kdSteering);

        get_parameter("kpThrottle", kpThrottle);
        get_parameter("kiThrottle", kiThrottle);
        get_parameter("kdThrottle", kdThrottle);
        get_parameter("kpSteering", kpSteering);
        get_parameter("kiSteering", kiSteering);
        get_parameter("kdSteering", kdSteering);

        targetThrottleSub = this->create_subscription<std_msgs::msg::Float64>(
            "throttle_command_raw", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg)
            {
                setpointThrottle = msg->data;
            });

        feedbackThrottleSub = this->create_subscription<std_msgs::msg::Float64>(
            "throttle", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg)
            {
                compute_pid_throttle(msg->data);
            });

        targetSteeringSub = this->create_subscription<std_msgs::msg::Float64>(
            "steering_command_raw", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg)
            {
                setpointSteering = msg->data;
            });

        feedbackSteeringSub = this->create_subscription<std_msgs::msg::Float64>(
            "steering", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg)
            {
                compute_pid_steering(msg->data);
            });

        steering_command_pub = this->create_publisher<std_msgs::msg::Float64>("steering_command", 10);
        throttle_command_pub = this->create_publisher<std_msgs::msg::Float64>("throttle_command", 10);

        lastTime = this->now();
    }

    void PIDControllerNode::compute_pid_steering(double current_value)
    {
        rclcpp::Time now = this->now();
        double dt = (now - lastTime).seconds();
        lastTime = now;

        double error = setpointSteering - current_value;
        integralSteering += error * dt;
        double derivative = (dt > 0.0) ? (error - prevErrorSteering) / dt : 0.0;

        double output = kpSteering * error + kiSteering * integralSteering + kdSteering * derivative;

        prevErrorSteering = error;

        auto msg = std_msgs::msg::Float64();
        msg.data = output;
        steering_command_pub->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Setpoint: %.2f | Feedback: %.2f | Output: %.2f",
                    setpointSteering, current_value, output);
    }

    void PIDControllerNode::compute_pid_throttle(double current_value)
    {
        rclcpp::Time now = this->now();
        double dt = (now - lastTime).seconds();
        lastTime = now;

        double error = setpointThrottle - current_value;
        integralThrottle += error * dt;
        double derivative = (dt > 0.0) ? (error - prevErrorThrottle) / dt : 0.0;

        double output = kpThrottle * error + kiThrottle * integralThrottle + kdThrottle * derivative;

        prevErrorThrottle = error;

        auto msg = std_msgs::msg::Float64();
        msg.data = output;
        throttle_command_pub->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Setpoint: %.2f | Feedback: %.2f | Output: %.2f",
                    setpointThrottle, current_value, output);
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pid_controller_node::PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
};
