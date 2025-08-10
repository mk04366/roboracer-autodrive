#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "control/stanley_controller_node.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "control/velocity_pid.hpp"
#include <fstream>
#include <sstream>

class StanleyNode : public rclcpp::Node
{
public:
    StanleyNode() : Node("stanley_controller_node"),
     velocity_pid_(1.0, 0.1, 0.05)
    {
        this->declare_parameter<std::string>("waypoints_path", "/home/ammar/ros2_ws/src/global-planning/outputs/map5/traj_race_cl.csv");
        this->declare_parameter<double>("k_path", 1.0);
        this->declare_parameter<double>("wheelbase", 0.33);

        std::string path = this->get_parameter("waypoints_path").as_string();
        k_path_ = this->get_parameter("k_path").as_double();
        double wheelbase = this->get_parameter("wheelbase").as_double();

        controller_ = std::make_unique<StanleyController>(wheelbase);

        if (!path.empty())
        {
            auto waypoints = loadWaypointsFromCSV(path);
            controller_->setWaypoints(waypoints);
        }

        ips_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/autodrive/f1tenth_1/ips", 10,
            std::bind(&StanleyNode::ipsCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu", 10,
            std::bind(&StanleyNode::imuCallback, this, std::placeholders::_1));

        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/speed", 10,
            std::bind(&StanleyNode::speedCallback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/waypoints_markers", 10);

        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle_command", 10);

        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/steering_command", 10);

        RCLCPP_INFO(this->get_logger(), "Stanley Controller Node Initialized");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ips_sub_;
    geometry_msgs::msg::Point last_position_;
    rclcpp::Time last_time_;
    bool first_ = true;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    std::unique_ptr<StanleyController> controller_;
    double k_path_;
    double smoothed_velocity_ = 0.0;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;

    double latest_yaw_ = 0.0;
    double latest_speed_ = 0.0;
    double velocity_integral_ = 0.0;
    double previous_velocity_error_ = 0.0;
    VelocityPID velocity_pid_;
    rclcpp::Time previous_time_;

    std::vector<Waypoint> loadWaypointsFromCSV(const std::string &filepath)
    {
        std::vector<Waypoint> waypoints;
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open waypoints file: " + filepath);
        }

        std::string line;
        while (std::getline(file, line))
        {
            // Skip comment or empty lines
            if (line.empty() || line[0] == '#')
                continue;

            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ';'))
            {
                // Trim whitespace
                token.erase(0, token.find_first_not_of(" \t\r\n"));
                token.erase(token.find_last_not_of(" \t\r\n") + 1);
                tokens.push_back(token);
            }

            if (tokens.size() < 6)
            {
                RCLCPP_WARN(this->get_logger(), "Skipping malformed line: '%s'", line.c_str());
                continue;
            }

            Waypoint wp;
            wp.x = std::stod(tokens[1]);
            wp.y = std::stod(tokens[2]);
            wp.heading = std::stod(tokens[3]);
            wp.velocity = std::stod(tokens[5]);
            waypoints.push_back(wp);
        }
        return waypoints;
    }

    void publishMarkers(size_t target_idx)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Publish all waypoints
        for (size_t i = 0; i < controller_->getWaypoints().size(); ++i)
        {
            const auto &wp = controller_->getWaypoints()[i];

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; // or "odom" if your CSV is in odometry frame
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "waypoints";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = wp.x;
            marker.pose.position.y = wp.y;
            marker.pose.position.z = 0.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2;

            if (i == target_idx)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
            }

            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration::from_seconds(0.2);

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto q = msg->orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        latest_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    void speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        latest_speed_ = msg->data;
    }

    void ipsCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        double x = msg->x;
        double y = msg->y;
        double yaw = latest_yaw_;
        double velocity = latest_speed_;

        try
        {
            double alpha = 0.2;
            smoothed_velocity_ = alpha * velocity + (1 - alpha) * smoothed_velocity_;

            auto [steering_angle, goal_speed] = controller_->plan(x, y, yaw, smoothed_velocity_, k_path_);

            auto now = this->get_clock()->now();

            // Compute throttle using PID velocity control
            double throttle_unclamped = velocity_pid_.compute(goal_speed, smoothed_velocity_, now);

            // Clamp throttle between 0 and 1
            double throttle_value = std::clamp(throttle_unclamped, 0.0, 1.0);

            // Publish drive message
            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = now;
            drive_msg.drive.steering_angle = steering_angle;
            drive_msg.drive.speed = goal_speed;
            drive_pub_->publish(drive_msg);

            // Steering command
            std_msgs::msg::Float32 steering_msg;
            steering_msg.data = static_cast<float>(steering_angle);
            steering_pub_->publish(steering_msg);

            // Throttle command
            std_msgs::msg::Float32 throttle_msg;
            throttle_msg.data = static_cast<float>(throttle_value);
            throttle_pub_->publish(throttle_msg);

            auto [_, __, target_idx, ___] = controller_->calcThetaAndEf({x, y, yaw, velocity}, controller_->getWaypoints());
            publishMarkers(target_idx);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyNode>());
    rclcpp::shutdown();
    return 0;
}
