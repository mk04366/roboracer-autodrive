#pragma once
#include <vector>
#include <Eigen/Dense>
#include <string>

namespace mpcc
{

    class Path
    {
    public:
        Path(const std::vector<Eigen::Vector4d> &waypoints,
             const std::vector<double> &time_profile,
             const std::vector<double> &velocities = std::vector<double>(),
             const std::vector<double> &velocities_y = std::vector<double>(),
             const std::vector<double> &psi_rate = std::vector<double>());
        size_t findNextWaypointIdx(double current_time, double horizon_time) const;
        Eigen::Vector2d getWaypoint(size_t index) const;
        double getHeading(size_t index) const;
        double getSteering(size_t index) const;
        double getTimeFromIndex(size_t index) const;
        size_t getTotalLength() const;
        double getVelocityX(size_t index) const;
        double getVelocityY(size_t index) const;
        double getPsiRate(size_t index) const;

    private:
        std::vector<Eigen::Vector4d> waypoints_;
        std::vector<double> time_profile_;
        std::vector<double> velocities_x_;
        std::vector<double> velocities_y_;
        std::vector<double> psi_rate_;
    };

    // Utility: load waypoints from CSV file
    Path load_path_from_csv(const std::string &filename);

} // namespace mpcc
