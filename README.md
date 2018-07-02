# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## 1. Your code should compile.
The code has been successfully compiled with no error. 

## 2. Implementation.
### 2.1 The Model
In this project, a dynamic model with optimization of objective function is used. The dynamic model has 6x states as listed below:
x: vehicle's position on x direction.
y: vehicle's position on y direction.
psi: vehicle's heading.
v: vehicle's velocity.
cte: cross track error.
epsi: vehicle's orientation error.

These states are updated via the dynamic model:
x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v_[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

and implemented in the MPC.cpp as shown below:
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

The output of the model are 2x variables, the throttle (acceleration) and steering (turning). In the main.cpp:
double steer = j[1]["steering_angle"];
double acc = j[1]["throttle"];

In the output, the steer value is then going to be divided by deg2rad(25) so that the range is in between [-1, 1].

The goal is to calculate the steer and throttle variable via optimizing the objective function, which is consist of 3x main parts.
-- part 1: square sum of error, to be minimized:
for (int i = 0; i < N; i++) {
  fg[0] += CTE_REF * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
  fg[0] += EPSI_REF * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
  fg[0] += V_REF * CppAD::pow(vars[v_start + i] - ref_v, 2);
}
-- part 2: square sum of actuator, to have bounded actions.
for (int i = 0; i < N-1; i++) {
  fg[0] += ST_FAC * CppAD::pow(vars[delta_start + i], 2);
  fg[0] += ACC_FAC * CppAD::pow(vars[a_start + i], 2);
}
-- part 3: square sum of value gap between sequential actuations, to have smooth output.
for (int i = 0; i < N-2; i++) {
  fg[0] += ST_SEQ * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
  fg[0] += ACC_SEQ * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}

The overall objective function is a sum of these three parts. Then to have a good model, I have spent a long time tuning this model, and below are some key changes that I have experienced to get the final result.

CTE_REF | EPSI_REF | V_REF | ST_FAC | ACC_FAC | ST_SEQ | ACC_SEQ | Results
--- | --- | --- | --- | --- | --- | --- | --- 
1 | 1 | 1 | 1 | 1 | 500 | 1 | first straight was ok, but too much OS% at turning. Failed.
1 | 1 | 1 | 1 | 1 | 50000 | 1 | a little better, but still failed.
1 | 1 | 1 | 1 | 1 | 300000 | 1 | much better, but still some obvious overshoot.
1 | 1 | 1 | 1 | 1 | 200000 | 1000 | almost finihsed, but cut the curb heavily.
1000 | 1000 | 1 | 1 | 1 | 200000 | 1000 | Unstable at turning. Speed was too fast.
1000 | 1000 | 1 | 50 | 50 | 200000 | 1000 | stable, but still some spikes.
1000 | 1000 | 1 | 100 | 100 | 200000 | 1000 | stable, but speed was too slow.
1000 | 1000 | 1 | 60 | 60 | 200000 | 1000 | stable and smooth. Chosen as final parameter.

So with those parameters tuned, optimizing the objective function provides steering and throttle to control the vehicle.

### 2.2 Timestep Length and Elapsed Duration (N & dt)
The N and dt values provides the consideration of prediction, which also impacts on the controller output. A too large N value will consider too many points in the future, and especially at sharp turns, this will make the output unstable. For example, during the time I firstly started with 25, it was very hard to make the sharp turn successful. I have tried from 25 down to 5 and finally chose N = 10, since a too small number will not be able to predict the dynamics ahead and also make the system oscillated. For the dt, I have tried from 0.05 to 0.25 where I can think this is within the stable range. I think 0.05 is a little too small to compensate the model, and dt = 0.1 gave a very good response. After dt above 0.25, the time gap is too long to handle the vehicle dynamics, and the system became unstable again. Overall, I used N = 10, dt = 0.1.

### 2.3 Polynomial Fitting and MPC Preprocessing
The waypoints provided in the simulator are first converted to vehicle coordinates, in main.cpp, shown as below: 
int n_ptsx = ptsx.size();
auto ptsx_vec = Eigen::VectorXd(n_ptsx);
auto ptsy_vec = Eigen::VectorXd(n_ptsx);
for (unsigned int i = 0; i < n_ptsx; i++) {
    double d_x = ptsx[i] - px;
    double d_y = ptsy[i] - py;
    double neg_psi = 0.0 - psi;
    ptsx_vec(i) = d_x * cos(neg_psi) - d_y * sin(neg_psi);
    ptsy_vec(i) = d_x * sin(neg_psi) + d_y * cos(neg_psi);
}

Then, a 3-rd order polynimial was used to fit. 
// fit poly
auto coeffs = polyfit(ptsx_vec, ptsy_vec, 3);

and the coefficients are then used to update the errors (cte and epsi) that are going to be used in MPC solver:
const double cte0 = coeffs[0];
const double epsi0 = -atan(coeffs[1]);

### 2.4 Model Predictive Control with Latency
The 100ms latency was considered by adding the latency part onto states. Multiplying the latency with corresponding velocity provides the latency part, and then they were added to the states, in the main.cpp, as shown below:
const int actuator_delay = 100;
const double delay = actuator_delay / 1000.0;

double steer = j[1]["steering_angle"];
double acc = j[1]["throttle"];

double x_delay = x0 + v * cos(psi0) * delay;
double y_delay = y0 + v * sin(psi0) * delay;
double psi_delay = psi0 - (v * steer * delay / Lf);
double v_delay = v + acc * delay;
double cte_delay = cte0 + v * sin(epsi0) * delay;
double epsi_delay = epsi0 - v * atan(coeffs[1]) * delay / Lf;

Then the states with delay considered are pushed into states to be solved.

## 3. The vehicle must successfully drive a lap around the track.
The vehicle successfully tracks the road a full lap and did not hit any curb. A recorded video (result_mov) is also included in this repo to check.

## 4. Appendix
forwarding original repo to run it.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
