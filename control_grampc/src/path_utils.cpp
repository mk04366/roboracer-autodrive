#include "control_grampc/path_utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

namespace mpcc
{

    Path::Path(const std::vector<Eigen::Vector4d> &waypoints)
        : waypoints_(waypoints), total_length_(0.0)
    {
        for (size_t i = 1; i < waypoints_.size(); ++i)
        {
            total_length_ += (waypoints_[i].head<2>() - waypoints_[i - 1].head<2>()).norm();
        }
    }

    double Path::total_length() const
    {
        return this->total_length_;
    }

    // New simplified interface implementation
    size_t Path::findClosestWaypoint(const Eigen::Vector2d& position) const
    {
        if (waypoints_.empty()) return 0;
        
        size_t closest_idx = 0;
        double min_distance = (position - waypoints_[0].head<2>()).norm();
        
        for (size_t i = 1; i < waypoints_.size(); ++i)
        {
            double distance = (position - waypoints_[i].head<2>()).norm();
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        return closest_idx;
    }

    Eigen::Vector2d Path::getWaypoint(size_t index) const
    {
        if (index >= waypoints_.size()) return waypoints_.back().head<2>();
        return waypoints_[index].head<2>();
    }

    double Path::getHeading(size_t index) const
    {
        if (index >= waypoints_.size()) return waypoints_.back()(2);
        return waypoints_[index](2);
    }

    double Path::getCurvature(size_t index) const
    {
        if (index >= waypoints_.size()) return waypoints_.back()(3);
        return waypoints_[index](3);
    }

    size_t Path::getWaypointCount() const
    {
        return waypoints_.size();
    }

    Path load_path_from_csv(const std::string &filename)
    {
        std::ifstream file(filename);
        std::string line;
        std::vector<Eigen::Vector4d> waypoints;

        while (std::getline(file, line))
        {
            // Skip comment lines
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            // Parse CSV with semicolon separators
            // Format: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ';'))
            {
                tokens.push_back(token);
            }

            if (tokens.size() >= 3)
            {
                try
                {
                    // double s = std::stod(tokens[0]); // arc length 
                    double x = std::stod(tokens[1]); // x coordinate
                    double y = std::stod(tokens[2]); // y coordinate
                    double heading = std::stod(tokens[3]); // heading
                    double curvature = std::stod(tokens[4]); // curvature
                    waypoints.emplace_back(x, y, heading, curvature);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
                }
            }
        }

        if (waypoints.size() < 2)
        {
            std::cerr << "Error: Not enough waypoints loaded from " << filename << std::endl;
        }

        return Path(waypoints);
    }

} // namespace mpcc
