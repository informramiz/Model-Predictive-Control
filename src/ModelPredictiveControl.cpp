////============================================================================
//// Name        : ModelPredictiveControl.cpp
//// Author      : Ramiz Raja
//// Version     :
//// Copyright   : MIT Licensed
//// Description : Hello World in C++, Ansi-style
////============================================================================
//
//#include <iostream>
//#include <cmath>
//#include "Eigen/Dense"
//
//using namespace std;
//using namespace Eigen;
//
////
//// Helper functions
////
//double pi() { return M_PI; }
//double deg2rad(double x) { return x * pi() / 180; }
//double rad2deg(double x) { return x * 180 / pi(); }
//
//const double Lf = 2;
//
//// Implement the global kinematic model.
//// Return the next state.
////
//// NOTE: state is [x, y, psi, v]
//// NOTE: actuators is [delta, a]
//Eigen::VectorXd GlobalKinematic(Eigen::VectorXd state,
//                                Eigen::VectorXd actuators,
//                                double dt);
//void TestGlobalKinematic();
//
//// Evaluate a polynomial.
//double polyeval(Eigen::VectorXd coeffs, double x);
//
//// Fit a polynomial.
//// Adapted from
//// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
//Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
//                        int order);
//void TestPolynomialFit();
//
//int main() {
//
//  //  TestGlobalKinematic();
//  TestPolynomialFit();
//}
//
//void TestGlobalKinematic() {
//  // [x, y, psi, v]
//  Eigen::VectorXd state(4);
//  // [delta, v]
//  Eigen::VectorXd actuators(2);
//
//  state << 0, 0, deg2rad(45), 1;
//  actuators << deg2rad(5), 1;
//
//  Eigen::VectorXd next_state = GlobalKinematic(state, actuators, 0.3);
//
//  std::cout << next_state << std::endl;
//}
//
//// Implement the global kinematic model.
//// Return the next state.
////
//// NOTE: state is [x, y, psi, v]
//// NOTE: actuators is [delta, a]
//Eigen::VectorXd GlobalKinematic(Eigen::VectorXd state,
//                                Eigen::VectorXd actuators,
//                                double dt) {
//  Eigen::VectorXd next_state(state.size());
//
//  //complete the next_state calculation ...
//  //get state variables
//  double x = state(0);
//  double y = state(1);
//  double yaw = state(2);
//  double v = state(3);
//
//  //get actuators
//  double delta = actuators(0); //steering angle
//  double a = actuators(1); //acceleration
//
//  next_state(0) = x + v * cos(yaw) * dt;
//  next_state(1) = y + v * sin(yaw) * dt;
//  next_state(2) = yaw + v/Lf * delta * dt;
//  next_state(3) = v + a * dt;
//
//  return next_state;
//}
//
//// Evaluate a polynomial.
//double polyeval(Eigen::VectorXd coeffs, double x) {
//  double result = 0.0;
//  for (int i = 0; i < coeffs.size(); i++) {
//    result += coeffs[i] * pow(x, i);
//  }
//  return result;
//}
//
//// Fit a polynomial.
//// Adapted from
//// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
//Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
//                        int order) {
//  assert(xvals.size() == yvals.size());
//  assert(order >= 1 && order <= xvals.size() - 1);
//  Eigen::MatrixXd A(xvals.size(), order + 1);
//
//  for (int i = 0; i < xvals.size(); i++) {
//    A(i, 0) = 1.0;
//  }
//
//  for (int j = 0; j < xvals.size(); j++) {
//    for (int i = 0; i < order; i++) {
//      A(j, i + 1) = A(j, i) * xvals(j);
//    }
//  }
//
//  auto Q = A.householderQr();
//  auto result = Q.solve(yvals);
//  return result;
//}
//
//void TestPolynomialFit() {
//  Eigen::VectorXd xvals(6);
//  Eigen::VectorXd yvals(6);
//  // x waypoint coordinates
//  xvals << 9.261977, -2.06803, -19.6663, -36.868, -51.6263, -66.3482;
//  // y waypoint coordinates
//  yvals << 5.17, -2.25, -15.306, -29.46, -42.85, -57.6116;
//
//  // use `polyfit` to fit a third order polynomial to the (x, y)
//  // coordinates.
//  Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);
//
//  for (double x = 0; x <= 20; x += 1.0) {
//    // use `polyeval` to evaluate the x values.
//    std::cout << polyeval(coeffs, x) << std::endl;
//  }
//
//  // Expected output
//  // -0.905562
//  // -0.226606
//  // 0.447594
//  // 1.11706
//  // 1.7818
//  // 2.44185
//  // 3.09723
//  // 3.74794
//  // 4.39402
//  // 5.03548
//  // 5.67235
//  // 6.30463
//  // 6.93236
//  // 7.55555
//  // 8.17423
//  // 8.7884
//  // 9.3981
//  // 10.0033
//  // 10.6041
//  // 11.2005
//  // 11.7925
//}
//
