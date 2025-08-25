#ifndef CONTROL_GRAMPC_MODEL_H
#define CONTROL_GRAMPC_MODEL_H

#include <Eigen/Dense>
#include "state.h"
#include "input.h"

namespace control_grampc {

class Model {
public:
    Model();
    virtual ~Model() = default;

    // Simulate dynamics (nonlinear forward simulation)
    void simulateDynamics(const State& state, const Input& input, double dt, State& new_state);
    
    // Getters for linearized matrices
    const Eigen::MatrixXd& A() const { return A_; }
    const Eigen::MatrixXd& B() const { return B_; }
    const Eigen::MatrixXd& C() const { return C_; }

private:
    Eigen::MatrixXd A_;  // State transition matrix
    Eigen::MatrixXd B_;  // Control input matrix  
    Eigen::MatrixXd C_;  // Offset vector
    
    static constexpr double CAR_LENGTH = 0.33;  // F1TENTH wheelbase
};

} // namespace control_grampc

#endif // CONTROL_GRAMPC_MODEL_H
