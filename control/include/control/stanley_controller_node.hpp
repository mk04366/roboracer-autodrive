#ifndef STANLEY_CONTROLLER_HPP
#define STANLEY_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <limits>
#include "control/common.hpp"

struct VehicleState
{
    double x;
    double y;
    double heading;
    double velocity;
};

class StanleyController
{
public:
    explicit StanleyController(double wheelbase = 0.33);

    void setWaypoints(const std::vector<Waypoint> &new_waypoints);
    const std::vector<Waypoint> getWaypoints();

    std::pair<double, double> plan(
        double pose_x,
        double pose_y,
        double pose_theta,
        double velocity,
        double k_path = 1.0,
        const std::vector<Waypoint> *override_waypoints = nullptr);

    double wheelbase_;
    std::vector<Waypoint> waypoints_;

    double pi2pi(double angle);
    std::tuple<double, double, size_t, double> calcThetaAndEf(
        const VehicleState &state,
        const std::vector<Waypoint> &waypoints);

    std::pair<double, double> controller(
        const VehicleState &state,
        const std::vector<Waypoint> &waypoints,
        double k_path);

    std::pair<size_t, std::array<double, 2>> nearestPoint(
        const std::array<double, 2> &point,
        const std::vector<Waypoint> &waypoints);

    
};

#endif // STANLEY_CONTROLLER_HPP
