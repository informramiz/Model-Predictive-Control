/*
 * main.cpp
 *
 *  Created on: May 20, 2017
 *      Author: ramiz
 */

#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen/Dense"
#include "MPC.h"
#include "json.hpp"
#include "matplotlibcpp.h"

// for convenience
namespace plt = matplotlibcpp;
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

int Test();
void MyTest();

void readLakeTrackCsv(std::vector<double> &x, std::vector<double> &y);
void TestLakeTrackCsv();

void TransformToVehicleCoordinates(double px, double py, double psi, vector<double> &ptsx, vector<double> &ptsy);
void TransformPointToVehicleCoordinates(double px, double py, double psi, double &x, double &y);
void TransformPointToVehicleCoordinates1(double px, double py, double psi, double &x, double &y);

void MyTransform() {
  double tx = -46.38823, ty = 66.63783;
  //  double psi = deg2rad(-45);
  double psi = 5.804719;

  Eigen::MatrixXd M(3, 3);
  //define vehicle to map
  M << cos(psi), -sin(psi), tx,
      sin(psi), cos(psi), ty,
      0, 0, 1;

  Eigen::VectorXd p(3);
  p << -61.09,92.88499, 1;
  auto t_p = M * p;

  //from map to vehicle is just inverse of M
  Eigen::MatrixXd M_inv = M.inverse();
  //  std::cout << M_inv << std::endl;
  auto p_o = M_inv * t_p;
  std::cout << p_o << std::endl;
}

int main() {
  //  Eigen::VectorXd v;
  //  std::vector<double> v1;
  //  v1.push_back(0);
  //  v = Eigen::VectorXd::Map(&v1[0], v1.size());
  //  return 0;

  return Test();
  //  TestLakeTrackCsv();

  //  MyTransform();
  return 0;
}

void some() {

}

void TestLakeTrackCsv() {
  std::vector<double> ptsx;
  std::vector<double> ptsy;
  readLakeTrackCsv(ptsx, ptsy);

  Eigen::VectorXd ptsx_eigen = Eigen::VectorXd::Map(&ptsx[0], ptsx.size());
  Eigen::VectorXd ptsy_eigen = Eigen::VectorXd::Map(&ptsy[0], ptsy.size());
  std::cout << "X points count: " << ptsx_eigen.size() << std::endl;
  std::cout << "Y points count: " << ptsy_eigen.size() << std::endl;

  Eigen::VectorXd coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);
  std::cout << "Polynomial coeffs count: " << coeffs.size() << std::endl;

  //  plt::plot(ptsx, ptsy, "r-", ptsx, [&coeffs](double x) { return coeffs[0] + coeffs[1] * x + coeffs[2] * pow(x, 2)
  //      + coeffs[3] * pow(x, 3); }, "k-");
  //  plt::show();

  MPC mpc;
  int iters = 50;

  double x = -1;
  double y = 10;
  double psi_des = 0;
  double psi = 0;
  double v = 10;

  double cte = polyeval(coeffs, x) - y;
  double derivative_f = coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * x * x;
  double epsi = psi_des - atan(derivative_f);

  Eigen::VectorXd state(6);
  state << x, y, psi, v, cte, epsi;

  std::vector<double> x_vals = {state[0]};
  std::vector<double> y_vals = {state[1]};
  std::vector<double> psi_vals = {state[2]};
  std::vector<double> v_vals = {state[3]};
  std::vector<double> cte_vals = {state[4]};
  std::vector<double> epsi_vals = {state[5]};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};

  for (size_t i = 0; i < iters; i++) {
    std::cout << "Iteration " << i << std::endl;

    //Display the MPC predicted trajectory
    vector<double> mpc_x_vals;
    vector<double> mpc_y_vals;
    auto vars = mpc.Solve(state, coeffs, mpc_x_vals, mpc_y_vals);

    x_vals.push_back(vars[0]);
    y_vals.push_back(vars[1]);
    psi_vals.push_back(vars[2]);
    v_vals.push_back(vars[3]);
    cte_vals.push_back(vars[4]);
    epsi_vals.push_back(vars[5]);

    delta_vals.push_back(vars[6]);
    a_vals.push_back(vars[7]);

    state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
    std::cout << state << std::endl;
  }

  plt::plot(ptsx, ptsy);
  plt::plot(x_vals, y_vals);
  plt::show();
}

void readLakeTrackCsv(std::vector<double> &ptsx, std::vector<double> &ptsy) {
  std::ifstream in_file("lake_track_waypoints.csv", std::ios::in);

  if(!in_file.is_open()) {
    std::cout << "Error - Unable to open CSV file" << std::endl;
    return;
  }

  int line_number = 0;
  std::string line;
  while(getline(in_file, line)) {
    line_number++;
    if (line_number < 2) {
      continue;
    }

    double x;
    char comma;
    double y;
    istringstream iss(line);
    iss >> x;
    iss >> comma;
    iss >> y;

    //    std::cout << x << ", " << y << std::endl;
    ptsx.push_back(x);
    ptsy.push_back(y);
  }

  in_file.close();
}

void MyTest() {
  MPC mpc;
  int iters = 50;

  Eigen::VectorXd ptsx(2);
  Eigen::VectorXd ptsy(2);
  ptsx << -100, 100;
  ptsy << -1, -1;

  // TODO: fit a polynomial to the above x and y coordinates
  // The polynomial is fitted to a straight line so a polynomial with
  // order 1 is sufficient.
  auto coeffs = polyfit(ptsx, ptsy, 1) ;

  // NOTE: free feel to play around with these
  double x = -1;
  double y = 10;
  double psi = 0;
  double v = 10;
  // TODO: calculate the cross track error
  // The cross track error is calculated by evaluating at polynomial at x, f(x)
  // and subtracting y --->: cte = f(x) - y
  double cte = polyeval(coeffs, 0) - y;

  // TODO: calculate the orientation error
  //the orientation error is epsi = psi - atan(f'(x)).
  //derivative of polynomial with order 1 is: (coeffs[0] + coeffs[1] * x) = coeffs[1]
  double epsi = 0 - atan(coeffs[1]);

  Eigen::VectorXd state(6);
  state << x, y, psi, v, cte, epsi;

  std::vector<double> x_vals = {state[0]};
  std::vector<double> y_vals = {state[1]};
  std::vector<double> psi_vals = {state[2]};
  std::vector<double> v_vals = {state[3]};
  std::vector<double> cte_vals = {state[4]};
  std::vector<double> epsi_vals = {state[5]};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};

  for (size_t i = 0; i < iters; i++) {
    std::cout << "Iteration " << i << std::endl;

    //Display the MPC predicted trajectory
    vector<double> mpc_x_vals;
    vector<double> mpc_y_vals;
    auto vars = mpc.Solve(state, coeffs, mpc_x_vals, mpc_y_vals);

    x_vals.push_back(vars[0]);
    y_vals.push_back(vars[1]);
    psi_vals.push_back(vars[2]);
    v_vals.push_back(vars[3]);
    cte_vals.push_back(vars[4]);
    epsi_vals.push_back(vars[5]);

    delta_vals.push_back(vars[6]);
    a_vals.push_back(vars[7]);

    state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
    std::cout << state << std::endl;
  }

  // Plot values
  // NOTE: feel free to play around with this.
  // It's useful for debugging!
  plt::subplot(3, 1, 1);
  plt::title("CTE");
  plt::plot(cte_vals);
  plt::subplot(3, 1, 2);
  plt::title("Delta (Radians)");
  plt::plot(delta_vals);
  plt::subplot(3, 1, 3);
  plt::title("Velocity");
  plt::plot(v_vals);

  plt::show();
}

int Test() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          //(x and y coordinates) path way points in map coordinates
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

//          std::cout << "X points count: " << ptsx.size() << std::endl;
//          std::cout << "Y points count: " << ptsy.size() << std::endl;

          //std::cout << "ptsx: " << ptsx[0] << ", " << ptsx[1] << std::endl;
          //std::cout << "x: " << px << std::endl;

          //current state of the vehicle in vehicle coordinates
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          //TODO: convert ptsx and ptsy to vehicle coordinates system
          TransformToVehicleCoordinates(px, py, psi, ptsx, ptsy);

          //TODO: Calculate CTE and epsi to complete vehicle state
          //fit  order 3 polynomial
          double x = 0;
          double psi_des = 0;
          auto coeffs = polyfit(Eigen::VectorXd::Map(&ptsx[0], ptsx.size()), Eigen::VectorXd::Map(&ptsy[0], ptsy.size()), 3);
          double cte = polyeval(coeffs, 0) - 0;

          /*
           * TODO: Calculate steering angle and throttle using MPC.
           *
           * Both are in between [-1, 1].
           *
           */
          double derivative_f = coeffs[1] + 2 * coeffs[2] * 0 + 3 * coeffs[3] * 0 * 0;
          double epsi = 0 - atan(derivative_f);

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          auto vars = mpc.Solve(state, coeffs, mpc_x_vals, mpc_y_vals);

          double steer_value = vars[6];
          double throttle_value = vars[7];

          json msgJson;
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

//          std::cout << "MPC_x count: " << mpc_x_vals.size() << std::endl;
//          std::cout << "MPC_y count: " << mpc_y_vals.size() << std::endl;
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = ptsx;
          vector<double> next_y_vals = ptsy;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
//          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

  return 0;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

void TransformToVehicleCoordinates(double px, double py, double psi, vector<double> &ptsx, vector<double> &ptsy) {
  size_t points_count = ptsx.size();
  for (int i = 0; i < points_count; ++i) {
    TransformPointToVehicleCoordinates1(px, py, psi, ptsx[i], ptsy[i]);
  }
}

void TransformPointToVehicleCoordinates(double px, double py, double psi, double &x, double &y) {
  //apply rotation to align vehicle coordinate system and map coordinate system
  //followed by translation to translate observation with respect to particle position
  x = x*cos(psi) - y*sin(psi) + px;
  y = x*sin(psi) + y*cos(psi) + py;
}

void TransformPointToVehicleCoordinates1(double px, double py, double psi, double &x, double &y) {
  //apply rotation to align vehicle coordinate system and map coordinate system
  //followed by translation to translate observation with respect to particle position
  //  x = x - px;
  //  y = y - py;
  //
  //  double angle = deg2rad(90);
  //  x = x*cos(psi) - y*sin(psi);
  //  y = x*sin(psi) + y*cos(psi);

  //  x = x + px;
  //  y = y + py;

  Eigen::VectorXd p(3);
  p << x, y, 1;

  Eigen::MatrixXd M(3, 3);
  M << cos(psi), -sin(psi), px,
      sin(psi), cos(psi), py,
      0, 0, 1;

  Eigen::MatrixXd M_inv = M.inverse();
  auto p_t = M_inv * p;

  x = p_t(0);
  y = p_t(1);
}

