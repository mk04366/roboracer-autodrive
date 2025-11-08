#include "control_grampc/path_utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

namespace mpcc
{

    Path::Path(const std::vector<Eigen::Vector4d> &waypoints,
               const std::vector<double> &time_profile,
               const std::vector<double> &velocities)
        : waypoints_(waypoints), time_profile_(time_profile), velocities_(velocities)
    {
    }
    size_t Path::findNextWaypointIdx(double current_time, double horizon_time) const
    {
        if (time_profile_.empty())
            return 0;

        double max_time = time_profile_.back();

        double target_time = current_time + horizon_time;

        if (target_time > max_time)
        {
            target_time = std::fmod(target_time, max_time);
        }

        for (size_t i = 0; i < time_profile_.size(); ++i)
        {
            if (time_profile_[i] >= target_time)
            {
                return i;
            }
        }

        return 0;
    }

    Eigen::Vector2d Path::getWaypoint(size_t index) const
    {
        if (index >= waypoints_.size())
            return waypoints_.back().head<2>();
        return waypoints_[index].head<2>();
    }

    double Path::getTimeFromIndex(size_t index) const
    {
        if (index >= time_profile_.size())
            return time_profile_.empty() ? 0.0 : time_profile_.back();
        return time_profile_[index];
    }

    double Path::getHeading(size_t index) const
    {
        if (index >= waypoints_.size())
            return waypoints_.back()(2);
        double adjusted_heading = waypoints_[index](2) + M_PI / 2.0; // adjust for Eigen frame
        // wrap to [-pi, pi]
        while (adjusted_heading > M_PI)
            adjusted_heading -= 2.0 * M_PI;
        while (adjusted_heading < -M_PI)
            adjusted_heading += 2.0 * M_PI;
        return adjusted_heading;
    }

    size_t Path::getTotalLength() const
    {
        return waypoints_.size();
    }

    double Path::getVelocity(size_t index) const
    {
        if (index >= velocities_.size())
            return velocities_.back();
        return velocities_[index];
    }

    double Path::getSteering(size_t index) const
    {
        if (index >= waypoints_.size())
            return waypoints_.back()(3);
        return waypoints_[index](3);
    }

    Path load_path_from_csv(const std::string &filename)
    {
        std::ifstream file(filename);
        std::string line;
        std::vector<Eigen::Vector4d> waypoints;
        std::vector<double> time_profile;
        std::vector<double> velocities;

        while (std::getline(file, line))
        {
            if (line.empty() || line[0] == '#')
                continue;

            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ','))
                tokens.push_back(token);

            if (tokens.size() >= 6) // we need at least 6 columns now
            {
                try
                {
                    double t = std::stod(tokens[0]);
                    double x = std::stod(tokens[1]);
                    double y = std::stod(tokens[2]);
                    double psi = std::stod(tokens[3]);
                    double steering = std::stod(tokens[4]);
                    double vx = std::stod(tokens[5]);

                    waypoints.emplace_back(x, y, psi, steering);
                    time_profile.push_back(t);
                    velocities.push_back(vx);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
                }
            }
        }

        return Path(waypoints, time_profile, velocities);
    }

} // namespace mpcc
