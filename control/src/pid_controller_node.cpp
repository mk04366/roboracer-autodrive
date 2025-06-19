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
        declare_and_get("kp_throttle", kp_throttle);
        declare_and_get("ki_throttle", ki_throttle);
        declare_and_get("kd_throttle", kd_throttle);
        declare_and_get("setpointSpeed", setpointSpeed);
        declare_and_get("integralSteering", integralSteering);
        declare_and_get("prevErrorSteering", prevErrorSteering);
        declare_and_get("integralThrottle", integralThrottle);
        declare_and_get("prevErrorThrottle", prevErrorThrottle);

        RCLCPP_INFO(
            this->get_logger(),
            "PID Controller Node initialized with parameters:\n"
            "kp_steering: %f, ki_steering: %f, kd_steering: %f"
            "kp_throttle: %f, ki_throttle: %f, kd_throttle: %f, setpointSpeed: %f\n"
            "integralSteering: %f, prevErrorSteering: %f, integralThrottle: %f, prevErrorThrottle: %f",
            kp_steering, ki_steering, kd_steering,
            kp_throttle, ki_throttle, kd_throttle, setpointSpeed,
            integralSteering, prevErrorSteering, integralThrottle, prevErrorThrottle);

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
        // Angles for side distances (in radians)
        float right_angle = -M_PI_2; // -90 degrees
        float left_angle = M_PI_2;   // +90 degrees

        // Convert angles to indices
        int right_index = static_cast<int>((right_angle - scan->angle_min) / scan->angle_increment);
        int left_index = static_cast<int>((left_angle - scan->angle_min) / scan->angle_increment);

        if (right_index < 0 || right_index >= static_cast<int>(scan->ranges.size()) ||
            left_index < 0 || left_index >= static_cast<int>(scan->ranges.size()))
            return;

        float right_dist = scan->ranges[right_index];
        float left_dist = scan->ranges[left_index];

        if (std::isnan(right_dist) || std::isinf(right_dist) ||
            std::isnan(left_dist) || std::isinf(left_dist))
            return;

        // The error is the difference between left and right distances
        wall_following_error = (left_dist - right_dist) - desired_center_offset;
    }

    void PIDControllerNode::compute_steering_control()
    {
        rclcpp::Time now = this->now();
        double dt = (now - lastTimeSteering).seconds();
        if (dt < 1e-6)
            return;

        lastTimeSteering = now;

        double error = wall_following_error;
        integralSteering += error * dt;

        // Clamp integral to prevent windup
        integralSteering = clamp(integralSteering, -1.0, 1.0);

        double derivative = (error - prevErrorSteering) / dt;
        prevErrorSteering = error;

        double output = kp_steering * error + ki_steering * integralSteering + kd_steering * derivative;

        output = clamp(output, -0.5, 0.5);

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
            setpointSpeed = 0.3;
        else
            setpointSpeed = 1.0;

        double error = setpointSpeed - feedbackSpeed;

        integralThrottle += error * dt;

        // Clamp integral to prevent windup
        integralThrottle = clamp(integralThrottle, -1.0, 1.0);

        double derivative = (error - prevErrorThrottle) / dt;
        prevErrorThrottle = error;

        double output = kp_throttle * error + ki_throttle * integralThrottle + kd_throttle * derivative;

        output = clamp(output, 0.0, 0.5);

        // Low-pass filter
        double alpha = 0.1; // Lower = smoother, but slower response (try between 0.05 and 0.3)
        smoothed_throttle_output = alpha * output + (1 - alpha) * smoothed_throttle_output;

        auto msg = std_msgs::msg::Float32();
        msg.data = static_cast<float>(smoothed_throttle_output);
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
