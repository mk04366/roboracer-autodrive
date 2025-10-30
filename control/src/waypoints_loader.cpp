#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "control/common.hpp" // For Waypoint struct
#include <fstream>
#include <sstream>

class WaypointsLoaderNode : public rclcpp::Node
{
public:
    WaypointsLoaderNode() : Node("waypoints_loader_node")
    {
        this->declare_parameter<std::string>(
            "waypoints_path",
            "/home/ammar/ros2_ws/src/global-planning/outputs/map5/traj_race_cl.csv");

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/waypoints_markers", 10);

        std::string path = this->get_parameter("waypoints_path").as_string();

        try
        {
            waypoints_ = loadWaypointsFromCSV(path);
            // RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());

            // for (size_t i = 0; i < waypoints_.size(); ++i)
            // {
            //     RCLCPP_INFO(this->get_logger(), "[%zu] x=%.3f, y=%.3f, heading=%.3f, vel=%.3f",
            //                 i, waypoints_[i].x, waypoints_[i].y, waypoints_[i].heading, waypoints_[i].velocity);
            // }

            // Publish markers continuously
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&WaypointsLoaderNode::publishMarkers, this));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s", e.what());
        }
    }

private:
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
            if (line.empty() || line[0] == '#')
                continue;

            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ';'))
            {
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

    void publishMarkers()
    {
        if (waypoints_.empty()) return;

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            const auto &wp = waypoints_[i];

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "waypoints";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = wp.x;
            marker.pose.position.y = wp.y;
            marker.pose.position.z = 0.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration(0, 0); // Infinite lifetime

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Waypoint> waypoints_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointsLoaderNode>());
    rclcpp::shutdown();
    return 0;
}
