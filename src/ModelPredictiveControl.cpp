//============================================================================
// Name        : ModelPredictiveControl.cpp
// Author      : Ramiz Raja
// Version     :
// Copyright   : MIT Licensed
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <cmath>
#include "Eigen/Dense"

using namespace std;

//
// Helper functions
//
double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;

// Implement the global kinematic model.
// Return the next state.
//
// NOTE: state is [x, y, psi, v]
// NOTE: actuators is [delta, a]
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators,
                                double dt) {
  Eigen::VectorXd next_state(state.size());

  //complete the next_state calculation ...
  //get state variables
  double x = state(0);
  double y = state(1);
  double yaw = state(2);
  double v = state(3);

  //get actuators
  double delta = actuators(0); //steering angle
  double a = actuators(1); //acceleration

  next_state(0) = x + v * cos(yaw) * dt;
  next_state(1) = y + v * sin(yaw) * dt;
  next_state(2) = yaw + v/Lf * delta * dt;
  next_state(3) = v + a * dt;

  return next_state;
}

int main() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  Eigen::VectorXd next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}
