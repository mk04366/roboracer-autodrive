#include "control_grampc/path_utils.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

namespace path
{
    static inline double hypot2(double dx, double dy) { return std::sqrt(dx * dx + dy * dy); }
    double wrap_angle(double a)
    {
        while (a > M_PI)
            a -= 2 * M_PI;
        while (a < -M_PI)
            a += 2 * M_PI;
        return a;
    }

    std::vector<RefPoint> load_centerline_csv(const std::string &csv_path)
    {
        std::vector<std::pair<double, double>> xy;
        std::ifstream f(csv_path);
        std::string line;
        if (!f.is_open())
            return {};
        while (std::getline(f, line))
        {
            if (line.empty())
                continue;
            std::stringstream ss(line);
            std::string xs, ys;
            if (!std::getline(ss, xs, ','))
                continue;
            if (!std::getline(ss, ys, ','))
                continue;
            xy.emplace_back(std::stod(xs), std::stod(ys));
        }
        if (xy.size() < 3)
            return {};
        // build s, theta, curvature (finite diff)
        std::vector<RefPoint> ref;
        ref.reserve(xy.size());
        double s = 0.0;
        for (size_t i = 0; i < xy.size(); ++i)
        {
            double x = xy[i].first, y = xy[i].second;
            double th = 0.0, k = 0.0;
            if (i + 1 < xy.size())
            {
                double dx = xy[i + 1].first - x, dy = xy[i + 1].second - y;
                th = std::atan2(dy, dx);
            }
            else
            {
                double dx = x - xy[i - 1].first, dy = y - xy[i - 1].second;
                th = std::atan2(dy, dx);
            }
            if (i > 0)
            {
                double dx = x - xy[i - 1].first, dy = y - xy[i - 1].second;
                s += hypot2(dx, dy);
            }
            ref.push_back({s, x, y, th, k});
        }
        // curvature via three-point formula
        for (size_t i = 1; i + 1 < ref.size(); ++i)
        {
            auto &A = ref[i - 1], &B = ref[i], &C = ref[i + 1];
            double ax = A.x, ay = A.y;
            double bx = B.x, by = B.y;
            double cx = C.x, cy = C.y;
            double a = hypot2(bx - cx, by - cy);
            double b = hypot2(ax - cx, ay - cy);
            double c = hypot2(ax - bx, ay - by);
            double s2 = (a + b + c) / 2.0;
            double area = std::max(1e-9, std::sqrt(std::max(0.0, s2 * (s2 - a) * (s2 - b) * (s2 - c))));
            double R = (a * b * c) / (4.0 * area);
            B.k = (R > 1e-6) ? (1.0 / R) : 0.0;
        }
        return ref;
    }

    static inline size_t clamp_index(size_t i, size_t n) { return (i < n ? i : n - 1); }

    void interp(const std::vector<RefPoint> &ref, double s, double &xr, double &yr, double &thr, double &kappa)
    {
        if (ref.empty())
        {
            xr = yr = thr = kappa = 0.0;
            return;
        }
        if (s <= ref.front().s)
        {
            xr = ref.front().x;
            yr = ref.front().y;
            thr = ref.front().th;
            kappa = ref.front().k;
            return;
        }
        if (s >= ref.back().s)
        {
            xr = ref.back().x;
            yr = ref.back().y;
            thr = ref.back().th;
            kappa = ref.back().k;
            return;
        }
        // linear search (can be replaced by binary search)
        size_t i = 1;
        while (i < ref.size() && ref[i].s < s)
            ++i;
        size_t i0 = i - 1, i1 = i;
        double s0 = ref[i0].s, s1 = ref[i1].s;
        double t = (s - s0) / std::max(1e-9, s1 - s0);
        xr = ref[i0].x + t * (ref[i1].x - ref[i0].x);
        yr = ref[i0].y + t * (ref[i1].y - ref[i0].y);
        double th0 = ref[i0].th, th1 = ref[i1].th;
        // unwrap small rotation interpolation
        double dth = wrap_angle(th1 - th0);
        thr = wrap_angle(th0 + t * dth);
        kappa = ref[i0].k + t * (ref[i1].k - ref[i0].k);
    }
}