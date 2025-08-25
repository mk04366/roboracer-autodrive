#include "control_grampc/model.h"
#include <cmath>

namespace control_grampc {

Model::Model() {
    A_ = Eigen::MatrixXd::Zero(4, 4);
    B_ = Eigen::MatrixXd::Zero(4, 2);
    C_ = Eigen::MatrixXd::Zero(4, 1);
}

void Model::simulateDynamics(const State& state, const Input& input, double dt, State& new_state) {
    // Extract current state
    double x     = state.x();
    double y     = state.y();
    double theta = state.theta();
    double v     = state.v();
    
    // Extract inputs
    double vel_cmd = input.velocity();
    double delta   = input.steeringAngle();
    
    // --- Nonlinear bicycle model dynamics ---
    double x_dot     = v * cos(theta);
    double y_dot     = v * sin(theta);
    double theta_dot = v * tan(delta) / CAR_LENGTH;
    double v_dot     = (vel_cmd - v) / 0.1;  // first-order velocity dynamics (lag)
    
    // --- Forward Euler integration ---
    new_state.setX(x + x_dot * dt);
    new_state.setY(y + y_dot * dt);
    new_state.setTheta(theta + theta_dot * dt);
    new_state.setV(v + v_dot * dt);
}

} // namespace control_grampc
