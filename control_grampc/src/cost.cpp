#include "control_grampc/cost.h"

namespace control_grampc {

Cost::Cost() {
    // Default cost matrices
    Q_.diagonal() << 10.0, 10.0, 1.0, 1.0;  // x, y, theta, v
    R_.diagonal() << 0.1, 1.0;              // velocity, steering
}

Cost::Cost(const Eigen::DiagonalMatrix<double, 4>& Q, 
           const Eigen::DiagonalMatrix<double, 2>& R) 
    : Q_(Q), R_(R) {}

} // namespace control_grampc
