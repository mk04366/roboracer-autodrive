#ifndef CONTROL_GRAMPC_COST_H
#define CONTROL_GRAMPC_COST_H

#include <Eigen/Dense>

namespace control_grampc {

class Cost {
public:
    Cost();
    Cost(const Eigen::DiagonalMatrix<double, 4>& Q, 
         const Eigen::DiagonalMatrix<double, 2>& R);
    virtual ~Cost() = default;

    // Getters
    const Eigen::DiagonalMatrix<double, 4>& Q() const { return Q_; }
    const Eigen::DiagonalMatrix<double, 2>& R() const { return R_; }
    
    // Setters
    void setQ(const Eigen::DiagonalMatrix<double, 4>& Q) { Q_ = Q; }
    void setR(const Eigen::DiagonalMatrix<double, 2>& R) { R_ = R; }

private:
    Eigen::DiagonalMatrix<double, 4> Q_;  // State cost matrix (x, y, theta, v)
    Eigen::DiagonalMatrix<double, 2> R_;  // Input cost matrix (velocity, steering)
};

} // namespace control_grampc

#endif // CONTROL_GRAMPC_COST_H
