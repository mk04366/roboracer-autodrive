#include "control_grampc/model.h"
#include <cmath>

namespace control_grampc {

Model::Model() {
    A_ = Eigen::MatrixXd::Zero(4, 4);
    B_ = Eigen::MatrixXd::Zero(4, 2);
    C_ = Eigen::MatrixXd::Zero(4, 1);
}

void Model::linearize(const State& state, const Input& input, double dt) {
    // Reset matrices
    A_ = Eigen::MatrixXd::Zero(4, 4);
    B_ = Eigen::MatrixXd::Zero(4, 2);
    C_ = Eigen::MatrixXd::Zero(4, 1);
    
    // Extract state variables
    double x = state.x();
    double y = state.y();
    double theta = state.theta();
    double v = state.v();
    
    // Extract input variables
    double vel = input.velocity();
    double delta = input.steeringAngle();
    
    // Linearize bicycle model around operating point
    // x_dot = v * cos(theta)
    // y_dot = v * sin(theta)  
    // theta_dot = v * tan(delta) / L
    // v_dot = acceleration (from velocity input)
    
    // State transition matrix A (Jacobian w.r.t. state)
    A_(0, 0) = 1.0;  // dx/dx
    A_(0, 2) = -vel * sin(theta) * dt;  // dx/dtheta
    A_(0, 3) = cos(theta) * dt;  // dx/dv
    
    A_(1, 1) = 1.0;  // dy/dy
    A_(1, 2) = vel * cos(theta) * dt;  // dy/dtheta
    A_(1, 3) = sin(theta) * dt;  // dy/dv
    
    A_(2, 2) = 1.0;  // dtheta/dtheta
    A_(2, 3) = tan(delta) * dt / CAR_LENGTH;  // dtheta/dv
    
    A_(3, 3) = 1.0;  // dv/dv (velocity is directly controlled)
    
    // Input matrix B (Jacobian w.r.t. input)
    B_(0, 0) = cos(theta) * dt;  // dx/dvel
    B_(1, 0) = sin(theta) * dt;  // dy/dvel
    B_(2, 0) = tan(delta) * dt / CAR_LENGTH;  // dtheta/dvel
    B_(2, 1) = vel * dt / (CAR_LENGTH * cos(delta) * cos(delta));  // dtheta/ddelta
    B_(3, 0) = 1.0;  // dv/dvel (direct velocity control)
    
    // Offset vector C (higher order terms)
    C_(0, 0) = vel * theta * sin(theta) * dt;
    C_(1, 0) = -vel * theta * cos(theta) * dt;
    C_(2, 0) = -delta * vel * dt / (CAR_LENGTH * cos(delta) * cos(delta));
    C_(3, 0) = 0.0;
}

void Model::simulateDynamics(const State& state, const Input& input, double dt, State& new_state) {
    // Extract current state
    double x = state.x();
    double y = state.y();
    double theta = state.theta();
    double v = state.v();
    
    // Extract inputs
    double vel_cmd = input.velocity();
    double delta = input.steeringAngle();
    
    // Bicycle model dynamics
    double x_dot = v * cos(theta);
    double y_dot = v * sin(theta);
    double theta_dot = v * tan(delta) / CAR_LENGTH;
    double v_dot = (vel_cmd - v) / 0.1;  // Simple first-order velocity dynamics
    
    // Forward Euler integration
    new_state.setX(x + x_dot * dt);
    new_state.setY(y + y_dot * dt);
    new_state.setTheta(theta + theta_dot * dt);
    new_state.setV(v + v_dot * dt);
}

} // namespace control_grampc
