#include "control/pid_controller_node.hpp"

namespace pid_controller_node
{

    PIDControllerNode::PIDControllerNode()
        : Node("pid_controller_node")
    {
        auto declare_and_get = [this](const std::string &name, double &value)
        {
            this->declare_parameter(name, value);
            this->get_parameter(name, value);
        };

        declare_and_get("kp_steering", kp_steering);
        declare_and_get("ki_steering", ki_steering);
        declare_and_get("kd_steering", kd_steering);
        declare_and_get("setpoint", setpoint);
        declare_and_get("kp_throttle", kp_throttle);
        declare_and_get("ki_throttle", ki_throttle);
        declare_and_get("kd_throttle", kd_throttle);
        declare_and_get("setpointSpeed", setpointSpeed);
        declare_and_get("integral", integral);
        declare_and_get("prevError", prevError);
        declare_and_get("integralThrottle", integralThrottle);

        RCLCPP_INFO(this->get_logger(), "PID Controller Node initialized with parameters:");
        RCLCPP_INFO(this->get_logger(), "kp_steering: %f", kp_steering);
        RCLCPP_INFO(this->get_logger(), "ki_steering: %f", ki_steering);
        RCLCPP_INFO(this->get_logger(), "kd_steering: %f", kd_steering);
        RCLCPP_INFO(this->get_logger(), "setpoint: %f", setpoint);
        RCLCPP_INFO(this->get_logger(), "kp_throttle: %f", kp_throttle);
        RCLCPP_INFO(this->get_logger(), "ki_throttle: %f", ki_throttle);
        RCLCPP_INFO(this->get_logger(), "kd_throttle: %f", kd_throttle);
        RCLCPP_INFO(this->get_logger(), "setpointSpeed: %f", setpointSpeed);
        RCLCPP_INFO(this->get_logger(), "integral: %f", integral);
        RCLCPP_INFO(this->get_logger(), "prevError: %f", prevError);
        RCLCPP_INFO(this->get_logger(), "integralThrottle: %f", integralThrottle);
        RCLCPP_INFO(this->get_logger(), "prevErrorThrottle: %f", prevErrorThrottle);

        feedbackSpeedSub = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/speed", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            {
                feedbackSpeed = msg->data;
            });

        feedbackLidarSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/autodrive/f1tenth_1/lidar", 10,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                process_lidar_scan(msg);
            });

        feedbackSteeringSub = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/steering", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            {
                feedbackSteering = msg->data;
            });

        feedbackThrottleSub = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            {
                feedbackThrottle = msg->data;
            });

        steering_command_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);
        throttle_command_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);

        lastTimeSteering = this->now();
        lastTimeThrottle = this->now();

        control_timer = this->create_wall_timer(
            std::chrono::milliseconds(56), // ~18 Hz = 1000 ms / 18 ≈ 56 ms
            [this]()
            {
                compute_steering_control();
                compute_throttle_control();
            });
    }

    void PIDControllerNode::process_lidar_scan(sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Example: right wall following
        // angle = -45 deg = -0.785 rad
        float desired_angle = -0.785f; // right side
        float desired_distance = 0.5f;

        int index = static_cast<int>((desired_angle - scan->angle_min) / scan->angle_increment);

        if (index < 0 || index >= static_cast<int>(scan->ranges.size()))
            return;

        float distance = scan->ranges[index];

        if (std::isnan(distance) || std::isinf(distance))
            return;

        float error = desired_distance - distance;

        // Save the error for steering PID
        wall_following_error = error;
        compute_steering_control();
    }

    void PIDControllerNode::compute_steering_control()
    {
        rclcpp::Time now = this->now();
        double dt = (now - lastTimeSteering).seconds();
        if (dt < 1e-6)
            return;

        lastTimeSteering = now;

        double error = wall_following_error;
        integral += error * dt;
        integral = clamp(integral, -1.0, 1.0);

        double derivative = (error - prevError) / dt;
        prevError = error;

        double output = kp_steering * error + ki_steering * integral + kd_steering * derivative;
        output = clamp(output, -0.4, 0.4); // steering angle limits (in radians)

        auto msg = std_msgs::msg::Float32();
        msg.data = static_cast<float>(output);
        steering_command_pub->publish(msg);
    }

    void PIDControllerNode::compute_throttle_control()
    {
        rclcpp::Time now = this->now();
        double dt = (now - lastTimeThrottle).seconds();
        if (dt < 1e-6)
            return;

        lastTimeThrottle = now;

        if (std::abs(wall_following_error) > 0.3)
            setpointSpeed = 0.5;
        else
            setpointSpeed = 1.0;

        double error = setpointSpeed - feedbackSpeed;

        integralThrottle += error * dt;
        integralThrottle = clamp(integralThrottle, -1.0, 1.0);

        double derivative = (error - prevErrorThrottle) / dt;
        prevErrorThrottle = error;

        double output = kp_throttle * error + ki_throttle * integralThrottle + kd_throttle * derivative;
        output = clamp(output, 0.0, 1.0); // throttle should be between 0 and 1

        auto msg = std_msgs::msg::Float32();
        msg.data = static_cast<float>(output);
        throttle_command_pub->publish(msg);
    }

} // namespace pid_controller_node

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pid_controller_node::PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}
