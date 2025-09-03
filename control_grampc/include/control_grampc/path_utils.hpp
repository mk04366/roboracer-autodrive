#pragma once
#include <vector>
#include <Eigen/Dense>
#include <string>

namespace mpcc
{

class Path
{
public:
    Path(const std::vector<Eigen::Vector4d> &waypoints);
    size_t findClosestWaypoint(const Eigen::Vector2d& position) const;
    Eigen::Vector2d getWaypoint(size_t index) const;
    double getHeading(size_t index) const;
    double getCurvature(size_t index) const;
    size_t getWaypointCount() const;
    double total_length() const;

private:
    std::vector<Eigen::Vector4d> waypoints_;
    double total_length_;
};

// Utility: load waypoints from CSV file
Path load_path_from_csv(const std::string &filename);

} // namespace mpcc
