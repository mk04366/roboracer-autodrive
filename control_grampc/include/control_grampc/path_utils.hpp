#pragma once
#include <vector>
#include <string>

namespace path {
struct RefPoint {
  double s;   // arc length [m]
  double x;   // [m]
  double y;   // [m]
  double th;  // tangent heading [rad]
  double k;   // curvature [1/m]
};

// Load CSV with columns: x,y (centerline). Builds arc-length and finite-diff theta/curvature.
std::vector<RefPoint> load_centerline_csv(const std::string& csv_path);

// Linear interpolation lookup. Assumes s within [0, s_end]. If closed track, wrap.
void interp(const std::vector<RefPoint>& ref, double s, double& xr, double& yr, double& thr, double& kappa);

double wrap_angle(double a);
}