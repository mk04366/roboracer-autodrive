#pragma once
#include <vector>
#include <Eigen/Dense>
#include <string>

namespace mpcc
{

class Path
{
public:
    Path(const std::vector<Eigen::Vector2d> &waypoints);

    Eigen::Vector2d interpolate(double s) const;
    double curvature(double s) const;
    double heading(double s) const;
    double total_length() const;

private:
    std::vector<Eigen::Vector2d> waypoints_;
    double total_length_;
};

// Utility: load waypoints from CSV file
Path load_path_from_csv(const std::string &filename);

} // namespace mpcc
