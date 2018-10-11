# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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

## Code Style

Stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## implementation

### Timestep length and elapsed duration (N, dt)

Large value of dt updates the control less frequently, then it makes harder to control accurately. Here, we set dt to 100ms, then N has been tuned appropriately. The N has been chosen as shown below considering the curvature of the road.

```
size_t N = 10;   // Number of time steps
double dt = 0.1; // 100 [msec]
```
### Control Delay of 100[ms] in model

The Model Predictive Control handles a 100 millisecond latency in the model by adding following lines.

```
px += v * cos(psi) * control_delay;
py += v * sin(psi) * control_delay;
cte += v * sin(epsi) * control_delay;
epsi += v * (-1) * steer_value * control_delay / Lf;
psi += v * (-1) * steer_value * control_delay / Lf;
// Assume acc is close to 0 during 0.1m range
//v += acc * control_delay;
```
### Tuning the Cost of MPC

The cost is saved in the first element of 'fg' vector, fg[0].
The first step of tuning is the cost factor of track error, 'cte'. If the this factor is too high, then vehicle is unstable oscillating at high speed, then the value has been reduced from initial value of 2000. Then, the cost of the steering differential, delta, has been tuned, such that adding higher value at higher speed making smoother control at high speed.

Following values have been chosen as the final cost function.

```
double costCte = 200; // Reduced from 2000 to reduce the oscillation
double costEpsi = 100;
double costV = 1;     // Set to 1 to avoid frequent brake
double costDeltaStart = 1000;
double costAStart = 10;

fg[0] = 0;
for (int i = 0; i < N; ++i) {
  fg[0] += costCte * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
  fg[0] += costEpsi * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
  fg[0] += costV * CppAD::pow(vars[v_start + i] - ref_v, 2);
}
for (int i = 0; i < N - 1; ++i) {
  fg[0] += 5 * CppAD::pow(vars[delta_start + i], 2);
  fg[0] += 5 * CppAD::pow(vars[a_start + i], 2);
}
for (int i = 0; i < N - 2; ++i) {
  fg[0] += (costDeltaStart * (vars[v_start + i] * 10 / costDeltaStart + 1)) * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
  fg[0] += costAStart * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}
```
