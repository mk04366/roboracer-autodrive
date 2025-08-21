#pragma once
#include <cmath>


namespace mpcc {
struct Params {
// Vehicle
double L = 0.33; // wheelbase [m]
double delta_max = 25.0 * M_PI/180.0; // max steer [rad]
double a_min = 0.0; // forbid reverse (>=0)
double a_max = 3.0; // max accel [m/s^2]
double v_max = 4.0; // max speed [m/s]
double a_lat_max = 3.0; // lateral accel limit [m/s^2]


// Horizon
double T = 0.5; // horizon [s]
int N = 12; // intervals


// Weights
double w_c = 100.0; // contour error
double w_l = 1.0; // lag error
double w_v = 1.0; // speed tracking
double w_u = 1.0; // input magnitude
double w_du = 10.0; // input rate
double w_terminal = 50.0; // terminal pos error
double kappa_gain = 5.0; // curvature speed gain


// Solver
int max_grad_iter = 12; // iterations per tick
};
}