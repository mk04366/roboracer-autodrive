#include "control/pid_controller_node.hpp"

namespace pid_controller_node
{

    PIDControllerNode::PIDControllerNode()
        : Node("pid_controller_node"),
          kiSteering(0.01), kdSteering(0.05), kpSteering(1.2),
          kiThrottle(0.01), kdThrottle(0.05), kpThrottle(1.5),
          setpointSteering(0.0), integralSteering(0.0), prevErrorSteering(0.0),
          setpointThrottle(0.0), integralThrottle(0.0), prevErrorThrottle(0.0)
    {
        auto declare_and_get = [this](const std::string &name, double &value)
        {
            this->declare_parameter(name, value);
            this->get_parameter(name, value);
        };

        declare_and_get("kpThrottle", kpThrottle);
        declare_and_get("kiThrottle", kiThrottle);
        declare_and_get("kdThrottle", kdThrottle);

        declare_and_get("kpSteering", kpSteering);
        declare_and_get("kiSteering", kiSteering);
        declare_and_get("kdSteering", kdSteering);

        RCLCPP_INFO(this->get_logger(), "PID Controller Node initialized with parameters: "
                                        "kpThrottle=%.2f, kiThrottle=%.2f, kdThrottle=%.2f, "
                                        "kpSteering=%.2f, kiSteering=%.2f, kdSteering=%.2f",
                    kpThrottle, kiThrottle, kdThrottle,
                    kpSteering, kiSteering, kdSteering);

        // given from the planning algorithm
        targetSteeringSub = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/steering_command_raw", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            {
                setpointSteering = msg->data;
            });

        // given from the planning algorithm
        targetThrottleSub = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle_command_raw", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            {
                setpointThrottle = msg->data;
            });

        // feedback from the simulator
        feedbackSteeringSub = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/steering", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            {
                compute_pid_steering(msg->data);
            });

        // feedback from the simulator
        feedbackThrottleSub = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            {
                compute_pid_throttle(msg->data);
            });

        // final control commands given to the simulator
        steering_command_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);
        throttle_command_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);

        lastTimeSteering = this->now();
        lastTimeThrottle = this->now();
    }

    void PIDControllerNode::compute_pid_steering(double current_value)
    {
        rclcpp::Time now = this->now();
        double dt = (now - lastTimeSteering).seconds();
        if (dt < 1e-6)
            return; // to avoid unstable derivative

        lastTimeSteering = now;

        const double error = setpointSteering - current_value;

        integralSteering += error * dt;
        integralSteering = clamp(integralSteering, -10.0, 10.0); 

        const double derivative = (error - prevErrorSteering) / dt;

        const double output = kpSteering * error + kiSteering * integralSteering + kdSteering * derivative;

        prevErrorSteering = error;

        auto msg = std_msgs::msg::Float32();
        msg.data = output;
        steering_command_pub->publish(msg);
    }

    void PIDControllerNode::compute_pid_throttle(double current_Throttle)
    {
        rclcpp::Time now = this->now();
        double dt = (now - lastTimeThrottle).seconds();
        if (dt < 1e-6)
            return; // to avoid unstable derivative

        lastTimeThrottle = now;

        const double error = setpointThrottle - current_Throttle;

        integralThrottle += error * dt;
        integralThrottle = clamp(integralThrottle, -10.0, 10.0); // Clamp integral to prevent windup
        const double derivative = (dt > 0.0) ? (error - prevErrorThrottle) / dt : 0.0;

        const double output = kpThrottle * error + kiThrottle * integralThrottle + kdThrottle * derivative;

        prevErrorThrottle = error;

        auto msg = std_msgs::msg::Float32();
        msg.data = output; // Throttle output
        throttle_command_pub->publish(msg);
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pid_controller_node::PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
};
