#include "control/ftg_controller_node.hpp"

namespace ftg_controller
{

    FTGController::FTGController() : Node("follow_gap_node")
    {
        this->declare_parameter("disparity_threshold", 0.5f);
        this->declare_parameter("bubble_radius", 10);
        this->declare_parameter("forward_speed", 0.5f);

        this->get_parameter("disparity_threshold", disparity_threshold_);
        this->get_parameter("bubble_radius", bubble_radius_);
        this->get_parameter("forward_speed", forward_speed_);

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/autodrive/f1tenth_1/lidar", 10,
            std::bind(&FTGController::lidar_callback, this, std::placeholders::_1));

        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
    }

    void FTGController::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto ranges = preprocess_lidar(msg->ranges);
        auto [start_idx, end_idx] = find_max_gap(ranges);
        int best_idx = find_best_point(start_idx, end_idx);
        drive_towards_point(best_idx, msg->angle_min, msg->angle_increment);
    }

    std::vector<float> FTGController::preprocess_lidar(const std::vector<float> &ranges)
    {
        std::vector<float> proc_ranges = ranges;

        for (size_t i = 1; i < ranges.size(); ++i)
        {
            // skip invalid ranges (scans which were not received)
            if (!std::isfinite(ranges[i]) || !std::isfinite(ranges[i - 1]))
                continue;

            // Calculate disparity between consecutive ranges
            float disparity = std::abs(ranges[i] - ranges[i - 1]);

            // If disparity exceeds threshold, mask the region around the closer point
            if (disparity > disparity_threshold_)
            {
                int closer_idx = (ranges[i] < ranges[i - 1]) ? i : i - 1; //either left or right point is closer
                int start = std::max(0, closer_idx - bubble_radius_);
                int end = std::min((int)ranges.size() - 1, closer_idx + bubble_radius_);

                //in this case, we just make the region zero instead of trimming their length
                for (int j = start; j <= end; ++j)
                    proc_ranges[j] = 0.0f; // Mask region
            }
        }

        return proc_ranges;
    }

    std::pair<int, int> FTGController::find_max_gap(const std::vector<float> &ranges)
    {
        // Find the longest continuous segment of valid ranges
        int max_start = 0, max_len = 0;
        int curr_start = -1, curr_len = 0;

        for (size_t i = 0; i < ranges.size(); ++i)
        {
            // Check if the range is valid (greater than 0.1) since others are already trimmed by us
            if (ranges[i] > 0.1f)
            {
                if (curr_start == -1)
                {
                    curr_start = i;
                    curr_len = 1;
                }
                else
                    curr_len++;
            }
            else
            {
                // if current segment is greater than max, update max
                if (curr_len > max_len)
                {
                    max_start = curr_start;
                    max_len = curr_len;
                }
                curr_start = -1;
                curr_len = 0;
            }
        }

        // Check at the end of the loop in case the longest segment ends at the last index
        if (curr_len > max_len)
        {
            max_start = curr_start;
            max_len = curr_len;
        }

        return {max_start, max_start + max_len - 1};
    }

    int FTGController::find_best_point(int start_idx, int end_idx)
    {
        //just get the middle point of the longest gap and this is our goal point
        return (start_idx + end_idx) / 2;
    }

    void FTGController::drive_towards_point(int best_point_idx, float angle_min, float angle_increment)
    {
        float angle = angle_min + best_point_idx * angle_increment;

        std_msgs::msg::Float32 steering_msg;
        std_msgs::msg::Float32 throttle_msg;

        steering_msg.data = clamp(angle, -0.4f, 0.4f); // Clip max steering
        throttle_msg.data = forward_speed_;

        steering_pub_->publish(steering_msg);
        throttle_pub_->publish(throttle_msg);
    }

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ftg_controller::FTGController>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}