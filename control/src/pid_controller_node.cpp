#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class PIDControllerNode : public rclcpp::Node {
public:
    PIDControllerNode()
    : Node("pid_controller_node"),
      kp_(1.0), ki_(0.0), kd_(0.1),
      setpoint_(0.0), integral_(0.0), prev_error_(0.0)
    {
        declare_parameter("kp", 1.0);
        declare_parameter("ki", 0.0);
        declare_parameter("kd", 0.1);
        get_parameter("kp", kp_);
        get_parameter("ki", ki_);
        get_parameter("kd", kd_);

        target_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "target", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                setpoint_ = msg->data;
            });

        feedback_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "feedback", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                compute_pid(msg->data);
            });

        control_pub_ = this->create_publisher<std_msgs::msg::Float64>("control", 10);

        last_time_ = this->now();
    }

private:
    void compute_pid(double current_value) {
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

        RCLCPP_INFO(this->get_logger(), "Setpoint: %.2f | Feedback: %.2f | Output: %.2f", setpoint_, current_value, output);
    }

    double kp_, ki_, kd_;
    double setpoint_, integral_, prev_error_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr feedback_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}
