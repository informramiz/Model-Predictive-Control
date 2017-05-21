# Model Predictive Control (MPC)

Model Predictive control coded in C++ to control a vehicle's `velocity` and `steering angle`, with a `latency of 100ms` in between vehicle commands. 

I have tuned this MPC implementation with following hyperparameter values.

```
(trajectory/horizon lenght)                                 --> N = 25
(timestep)                                                  --> dt = 0.03
reference velocity                                          --> ref_v = 40 mph
(distance between vehicle front and center of gravity)      --> Lf = 2.67
(steering angle change rate penality in cost function)      --> 10000
```

Following is a part of video recorded by running MPC with above mentioned hyperparameter values.

![animation](visualization/animated.gif)

## Getting Started

There is a detailed example of code in main.cpp that you can refer to for how to connect to the simulator and how to use MPC. Following is a high level use of MPC code.

```

// MPC is initialized here!
MPC mpc;

//Required input: path points ptsx, ptsy and 
//vehicle position (px, py), vehicle orientation psi and vehicle speed v

//(x and y coordinates) path way points in map coordinates
vector<double> ptsx = ?
vector<double> ptsy = ?

//current state of the vehicle in vehicle coordinates
double px = ? ;
double py = ? ;
double psi = ? ;
double v = ? ;

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

//convert std::vector to Eigen::VectorXd
Eigen::VectorXd ptsx_eigen = Eigen::VectorXd::Map(&ptsx[0], ptsx.size());
Eigen::VectorXd ptsy_eigen = Eigen::VectorXd::Map(&ptsy[0], ptsy.size());

//fit a 3rd order polynomial to form reference line. Most of the roads in the world
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

//pass steer_value and throttel_value to vehicle

```


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions

There are scripts for `clean.sh`, `build.sh` and `run.sh` and a script `do.sh` to do build and run together or you can compile the project manually using following cmake commands.  

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

