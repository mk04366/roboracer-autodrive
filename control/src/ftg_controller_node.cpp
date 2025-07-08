#include "rclcpp/rclcpp.hpp"
#include "control/common.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>

using std::placeholders::_1;

class FTGNode : public rclcpp::Node
{
private:
    float last_steering_angle_ = 0.0f;
    float kd_gain_ = 0.7f;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<float> ranges = msg->ranges;
        auto processed = preprocess_lidar(ranges);
        auto [start, end] = find_max_gap(processed);
        int mid = find_midpoint(start, end);
        float steering = calculate_steering_angle(mid, ranges.size());
        float throttle = calculate_throttle(steering, processed);
        publish_steering(steering);
        publish_throttle(throttle);
    }

    std::vector<float> preprocess_lidar(const std::vector<float> &ranges)
    {
        const float disparity_threshold = 0.3f; // Minimum difference to trigger a disparity
        const int bubble_radius = 3;            // How many indices to erase near a disparity
        std::vector<float> processed = ranges;

        for (size_t i = 0; i < ranges.size() - 1; ++i)
        {
            float d1 = ranges[i];
            float d2 = ranges[i + 1];

            if (std::abs(d1 - d2) > disparity_threshold)
            {
                if (d1 < d2)
                {
                    // Obstacle on the left side — erase to the right
                    for (int j = 1; j <= bubble_radius && (i + j) < processed.size(); ++j)
                    {
                        processed[i + j] = std::min(processed[i + j], d1);
                    }
                }
                else
                {
                    // Obstacle on the right side — erase to the left
                    for (int j = 0; j <= bubble_radius && j <= static_cast<int>(i); ++j)
                    {
                        processed[i - j] = std::min(processed[i - j], d2);
                    }
                }
            }
        }

        return processed;
    }

    std::pair<int, int> find_max_gap(const std::vector<float> &ranges)
    {
        int max_gap = 0, start_gap = 0, end_gap = 0, curr = 0;
        for (size_t i = 0; i < ranges.size(); ++i)
        {
            if (ranges[i] > 1.23f)
            {
                ++curr;
                if (curr > max_gap)
                {
                    max_gap = curr;
                    end_gap = i;
                    start_gap = i - max_gap + 1;
                }
            }
            else
            {
                curr = 0;
            }
        }
        return {start_gap, end_gap};
    }

    int find_midpoint(int start, int end)
    {
        return (start + end) / 2;
    }

    float calculate_steering_angle(int best_point, int total_points)
    {
        float angle_increment = static_cast<float>(M_PI) / static_cast<float>(total_points);
        float raw_steering = (static_cast<float>(best_point) - static_cast<float>(total_points) / 2.0f) * angle_increment;

        // Clamp raw steering
        const float max_steering = 1.0f;
        raw_steering = clamp(raw_steering, -max_steering, max_steering);

        // Derivative term
        float delta = raw_steering - last_steering_angle_;
        float corrected_steering = raw_steering + kd_gain_ * delta;

        // Clamp again
        corrected_steering = clamp(corrected_steering, -max_steering, max_steering);

        // Update memory
        last_steering_angle_ = corrected_steering;

        return corrected_steering;
    }

    float calculate_throttle(float steering_angle, const std::vector<float> &ranges)
    {
        float target_pct = 0.05f;
        int mid = ranges.size() / 2;
        int half_window = static_cast<int>(ranges.size() * target_pct / 2);
        float center_avg = std::accumulate(ranges.begin() + mid - half_window,
                                           ranges.begin() + mid + half_window, 0.0f) /
                           (2 * half_window);
        float min_dist = *std::min_element(ranges.begin(), ranges.end());
        float base_throttle = 0.0f;

        if (center_avg > 7.0f && std::abs(steering_angle) < 0.1f)
        {
            if (center_avg > 10.0f)
                center_avg = 10.0f;
            base_throttle = 0.5f * (center_avg + 0.01f) / 10.0f;
        }
        else
        {
            if (min_dist > 0.6f)
                base_throttle = 0.15f;
            else if (min_dist > 0.5f)
                base_throttle = 0.3f;
            else if (min_dist > 0.4f)
                base_throttle = 0.2f;
            else if (min_dist > 0.12f)
                base_throttle = 0.15f;
            else if (min_dist > 0.09f)
                base_throttle = 0.04f;
            else
                base_throttle = 0.01f;
        }

        float decay = 0.9f;
        float throttle = decay * base_throttle * (1 - std::abs(steering_angle) + THROTTLE_ADJUSTMENT_FACTOR);
        RCLCPP_INFO(this->get_logger(), "Throttle out: %f", throttle);
        return throttle;
    }

public:
    FTGNode() : Node("ftg_node")
    {
        RCLCPP_INFO(this->get_logger(), "Awaiting subscriptions, ensure bridge is running");

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/autodrive/f1tenth_1/lidar", 10, std::bind(&FTGNode::lidar_callback, this, _1));

        steer_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
    }

    void publish_steering(float angle)
    {
        auto msg = std_msgs::msg::Float32();
        msg.data = angle;
        steer_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Steering successfully published: %f", angle);
    }

    void publish_throttle(float value)
    {
        auto msg = std_msgs::msg::Float32();
        msg.data = value;
        throttle_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Throttle successfully published: %f", value);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FTGNode>();

    rclcpp::spin(node);

    // On shutdown: publish 0 to stop the car
    auto msg = std_msgs::msg::Float32();
    msg.data = 0.0f;
    node->publish_throttle(msg.data);
    node->publish_steering(msg.data);

    rclcpp::shutdown();
    return 0;
}
