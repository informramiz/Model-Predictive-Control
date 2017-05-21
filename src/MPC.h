#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state.
  // Return the next state and actuations as a
  // vector.
  vector<double> Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs, vector<double> &x_vals, vector<double> &y_vals);
};

#endif /* MPC_H */
