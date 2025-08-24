#include "control_grampc/state.h"

namespace control_grampc {

State::State() : x_(0.0), y_(0.0), theta_(0.0), v_(0.0) {}

State::State(double x, double y, double theta) 
    : x_(x), y_(y), theta_(theta), v_(0.0) {}

State::State(double x, double y, double theta, double v) 
    : x_(x), y_(y), theta_(theta), v_(v) {}

Eigen::VectorXd State::toVector() const {
    Eigen::VectorXd vec(size_);
    vec << x_, y_, theta_, v_;
    return vec;
}

// Definition of static member
const int State::size_;

} // namespace control_grampc
