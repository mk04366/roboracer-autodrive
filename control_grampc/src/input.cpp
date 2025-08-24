#include "control_grampc/input.h"

namespace control_grampc {

Input::Input() : velocity_(0.0), steering_angle_(0.0) {}

Input::Input(double velocity, double steering_angle) 
    : velocity_(velocity), steering_angle_(steering_angle) {}

Eigen::VectorXd Input::toVector() const {
    Eigen::VectorXd vec(size_);
    vec << velocity_, steering_angle_;
    return vec;
}

// Definition of static member
const int Input::size_;

} // namespace control_grampc
