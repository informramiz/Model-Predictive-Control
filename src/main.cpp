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
string HasData(string s);

// Evaluate a polynomial.
double PolyEval(Eigen::VectorXd coeffs, double x);
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

// code for my own testing of MPC
void MyTest();
//function to read provided CSV file
void ReadLakeTrackCsv(std::vector<double> &x, std::vector<double> &y);
//function to test MPC on lake
void TestLakeTrackCsv();
//function to conver way points from Map to vehicle coordinates
void TransformToVehicleCoordinates(double px, double py, double psi, vector<double> &ptsx, vector<double> &ptsy);
//function to transform a single point from Map to vehicle coordinates
void TransformPointToVehicleCoordinates(double px, double py, double psi, double &x, double &y);

//Code provided by Udacity to pass project criteria
int UdacityTest();

int main() {
  return UdacityTest();
}

void TestLakeTrackCsv() {
  std::vector<double> ptsx;
  std::vector<double> ptsy;
  ReadLakeTrackCsv(ptsx, ptsy);

  Eigen::VectorXd ptsx_eigen = Eigen::VectorXd::Map(&ptsx[0], ptsx.size());
  Eigen::VectorXd ptsy_eigen = Eigen::VectorXd::Map(&ptsy[0], ptsy.size());
  std::cout << "X points count: " << ptsx_eigen.size() << std::endl;
  std::cout << "Y points count: " << ptsy_eigen.size() << std::endl;

  Eigen::VectorXd coeffs = Polyfit(ptsx_eigen, ptsy_eigen, 3);
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

  double cte = PolyEval(coeffs, x) - y;
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

void ReadLakeTrackCsv(std::vector<double> &ptsx, std::vector<double> &ptsy) {
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
  auto coeffs = Polyfit(ptsx, ptsy, 1) ;

  // NOTE: free feel to play around with these
  double x = -1;
  double y = 10;
  double psi = 0;
  double v = 10;
  // TODO: calculate the cross track error
  // The cross track error is calculated by evaluating at polynomial at x, f(x)
  // and subtracting y --->: cte = f(x) - y
  double cte = PolyEval(coeffs, 0) - y;

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

int UdacityTest() {
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
      string s = HasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          //(x and y coordinates) path way points in map coordinates
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          //current state of the vehicle in vehicle coordinates
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          //convert ptsx and ptsy to vehicle coordinates system
          TransformToVehicleCoordinates(px, py, psi, ptsx, ptsy);

          /***************
           * From here onward, all coordinates are in vehicle coordinate system
           ***************/

          //Note: in vehicle coordinates vehicle position (px, py) is actually origin so (px, py) = (0, 0)
          //similarly orientation of vehicle in vehicle coordinates is 0
          px = 0;
          py = 0;
          psi = 0;

          //convert vector to Eigen::VectorXd
          Eigen::VectorXd ptsx_eigen = Eigen::VectorXd::Map(&ptsx[0], ptsx.size());
          Eigen::VectorXd ptsy_eigen = Eigen::VectorXd::Map(&ptsy[0], ptsy.size());

          //fit a 3rd order polynomial to form reference line. Most of the road in world
          //can be represented by a 3rd order polynomial
          auto coeffs = Polyfit(ptsx_eigen, ptsy_eigen, 3);
          //calculate cross track error of vehicle position in y-axis (vehicle coordinates
          double cte = PolyEval(coeffs, px) - py;

          /*
           * Calculate steering angle and throttle using MPC.
           *
           * Both are in between [-1, 1].
           *
           */
          //calculate derivative of 3rd order polynomial fitted on px
          double derivative_f = coeffs[1] + 2 * coeffs[2] * px + 3 * coeffs[3] * px * px;
          //calculate desired orientation (orientation of reference line)
          double psi_desired = atan(derivative_f);
          //calculate orientation error epsi between reference line orientation (psi_desired)
          //and vehicle orientation psi
          double epsi = psi - psi_desired;

          //make state vector
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          //vectors to store the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //call MPC solver with current vehicle state
          auto vars = mpc.Solve(state, coeffs, mpc_x_vals, mpc_y_vals);
          //extract steering angle and throttle which are at 6 and 7 indexes
          double steer_value = vars[6];
          double throttle_value = vars[7];

          json msgJson;
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

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
          this_thread::sleep_for(chrono::milliseconds(100));
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
string HasData(string s) {
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
double PolyEval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
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
  //convert point to homogeneous space to apply transformation
  Eigen::VectorXd p(3);
  p << x, y, 1;

  //define vehicle to map coordinate system transformation which is:
  //apply rotation to align vehicle coordinate system and map coordinate system
  //followed by translation to translate observation with respect to vehicle position
  //result will be in map coordinates
  Eigen::MatrixXd M(3, 3);
  M << cos(psi), -sin(psi), px,
      sin(psi), cos(psi), py,
      0, 0, 1;


  //transformation for converting from map-coordinate system to
  //vehicle-coordinate system is just inverse of transform M (which vehicle to map transformation)
  Eigen::MatrixXd M_inv = M.inverse();
  //transform the point to vehicle coordinates
  auto p_t = M_inv * p;

  //dehomogenize the point by removing the last value which is equal to 1
  x = p_t(0);
  y = p_t(1);
}

