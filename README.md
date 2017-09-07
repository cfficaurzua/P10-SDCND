# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
The objective of this project is to implement an MPC controller using c++ to drive a car through a simulator build in unity.

the MPC controller takes into acount a model of the car dynamics, to predict the next states into the future and take the best actuator parameters option.

### Car state
In this project the car is modeled with a position (px, py), orientation (psi), and velocity (v). also the state includes the cross track error (cte) which is the difference between the actual position of the car and the desired position and a orientation error (epsi) which is the difference between the actual orientation of the car and the desired orientation, the desired position and orientation comes from a reference path, in reality this path could come from a path planning algorithm from the current location to a destination.

x = [ px, py, v, psi, cte, epsi]

### Actuators
to drive the car, there are only two actuators the steering (δ) and throttle (a),
the steering is limited to ±25° while the throttle is bounded to ±1, where negative values corresponds to braking or going backwards if the car is already stopped.

actuators = [δ,a]

### Update
The next state of the car is obtained from the current state, with the following non-linear equations:

* px(t+1) = x(t) + v(t)*cos(psi)*dt
* py(t+1) = y(t) + v(t)*sin(psi)*dt
* psi(t+1) = psi(t) + v(t)/Lf*δ(t)*dt
* v(t+1) = v(t) + a*dt
* cte(t+1) = cte(t) + v(t)*sin(epsi)*dt
* epsi(t+1) = epsi(t) + v(t)/Lf*δ(t)*dt

### parameters
The parameters to be tuned are dt which corresponde the interval between each step where the actuators values are predicted, and N, that correponds the number of steps into the future that the optimization algorithm takes into account.
dt was set to 0.1, and N was set to 20, at first I chose 0.1 and 10, but as the car speeded up, it failed to keep on track, which makes sense, the faster you go, you'd need to look more ahead.

### Cost Function
to found a local minima, the optimizer needs a cost function.
the cost function I chose emphasize in first place to drive smooth, then to maintained the reference orientation as close as posible, as well as being centered, then to use minimum throttle and steering use, finally to maintain a reference speed of 40 mph.

the cost function is then the following:

cost = 400 * cte(t)^2 + 2000 * epsi(t)^2 + (v(t) - ref_v)^2 + 500 * δ(t)^2 + 100*a(t)^2 + 2000 * (δ(t+1)-δ(t))^2 + 1000 * (a(t+1)-a(t))^2

### Result

The video below shows how the MPC controller performs in the driving track

### Conclusions

The MPC controller achieves the goal of driving within the track for at least one lap, but in the curves sometimes it woobles and brakes with non natural manuevering. one reason could be that the optimizer couldn't found the optimal solution in less than 100ms, a solution I can think of, is to apply the actuator results from the previous solution if the optimizer couldn't find a local optima within the range of time. in the future I would also like to add to the cost function, a function to keep the reference velocity only if the steering is close to zero, so It would take the curves with precaution. I tried this multiplying (v(t) - ref_v)^2 by d(t), but the car started to go backwards. I would dig in this problem in the future.

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

