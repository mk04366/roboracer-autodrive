#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "control/stanley_controller_node.hpp"

#include <fstream>
#include <sstream>

class StanleyNode : public rclcpp::Node {
public:
    StanleyNode() : Node("stanley_controller_node") {
        this->declare_parameter<std::string>("waypoints_path", "");
        this->declare_parameter<double>("k_path", 5.0);
        this->declare_parameter<double>("wheelbase", 0.33);

        std::string path = this->get_parameter("waypoints_path").as_string();
        k_path_ = this->get_parameter("k_path").as_double();
        double wheelbase = this->get_parameter("wheelbase").as_double();

        controller_ = std::make_unique<StanleyController>(wheelbase);

        if (!path.empty()) {
            auto waypoints = loadWaypointsFromCSV(path);
            controller_->setWaypoints(waypoints);
        }

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&StanleyNode::odomCallback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        RCLCPP_INFO(this->get_logger(), "Stanley Controller Node Initialized");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    std::unique_ptr<StanleyController> controller_;
    double k_path_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // Convert quaternion to yaw
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        double velocity = msg->twist.twist.linear.x;

        try {
            auto [steering_angle, speed] = controller_->plan(x, y, yaw, velocity, k_path_);

            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = this->get_clock()->now();
            drive_msg.drive.steering_angle = steering_angle;
            drive_msg.drive.speed = speed;

            drive_pub_->publish(drive_msg);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Planning failed: %s", e.what());
        }
    }

    std::vector<Waypoint> loadWaypointsFromCSV(const std::string& filepath) {
        std::vector<Waypoint> waypoints;
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open waypoints file: " + filepath);
        }

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            Waypoint wp;
            std::getline(ss, token, ','); wp.x = std::stod(token);
            std::getline(ss, token, ','); wp.y = std::stod(token);
            std::getline(ss, token, ','); wp.velocity = std::stod(token);
            std::getline(ss, token, ','); wp.heading = std::stod(token);
            waypoints.push_back(wp);
        }
        return waypoints;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyNode>());
    rclcpp::shutdown();
    return 0;
}
