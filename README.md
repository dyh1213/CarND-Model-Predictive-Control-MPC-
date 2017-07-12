# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

My implementation of the Udacity self-driving car engineer MPC project enables the car to drive up to 100mph smoothly around the track.

You can find a video of the car driving here:
https://www.youtube.com/watch?v=Bli75D95T3U&feature=youtu.be

A rule based target speed algorithm calculates the complexity of an upcoming turn, and is able to derive when the car should slow its target velocity (prior to the turn), and when its safe to accelerate again (generally out of the turn).

My ouput console lists 3 parameters.

* The Turn_Complexity which targets the following velocities in the MPC
	
	Note: this code section was removed for now as I needed to make some changes before the project deadline :(

    ```
	double targetV;
	targetV = 80 - (totalCIS * 2);
	auto vars = mpc.Solve(state, coeffs, targetV)
    ```

* The throttle which fluctuates between 1 and -1

* The angle

Vehicle Model

The vehicle model used in this project is based on the kinematic bicycle model. It does not take into account dynamic effects such as inertia, friction and torque. 

It is based on the following variables as given in the class lessons:
	
	...
	fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
	fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
	fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
	fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
	fg[2 + cte_start + i] =
			cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
	fg[2 + epsi_start + i] =
			epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
	...

Here, x,y denote the position of the car, psi the heading direction, v its velocity cte the cross-track error and epsi the orientation error. Lf is the distance between the center of mass of the vehicle and the front wheels and affects the maneuverability. The vehicle model can be found in the class FG_eval.

Latency

The latency problem is solved by assuming the car will drive at the current speed and direction before receiving any control input from the MPC by simply sleeping the thread before sending to the simulator (main.cpp line 240 - his_thread::sleep_for(chrono::milliseconds(100));)

The car input data is adjusted to this latenct by calculating the future x,y,psi, and velocity after the 100ms assuming all other things stay the same.

	...
	const double delay = 0.1; //ms
	const double Lf = 2.67;

	//Using the kinematic model
	const double delayed_px = px + speed_mph * cos(psi) * delay;
	const double delayed_py = py + speed_mph * sin(psi) * delay;
	const double delayed_psi = psi + (speed_mph * tan(-delta) / Lf) * delay + ((a * tan(-delta) / (2 * Lf)) * pow(delay, 2));
	const double delayed_v = speed_mph + a * delay;
	...

Dt & N

Through manual testing I found that a tighter (in my case 0.01 = 10ms) results in better results than a longer N (in my case 50). In comparison to the real world, it is better to perfect your current action then set up for an optimal action in the future. My car plots its best path for a mere 0.5 seconds ahead.
	
In the example below one can see that the turn complexity spikes up prior to the curve, with the MOC responding by slowing the car by changing the throttle to -1. The car then accelaretes again as the velocity is now lower than the target velocity fed to the MPC.

Preprocessing and Polynomial Fitting

The simulator coordinates are converted to our vehicle coordinates by calculating the distance from the original point to the current point, and then angle in the simulator space and converting it to vehicle space.

	...
	assert(ptsx.size() == ptsy.size());
	unsigned len = ptsx.size();

	auto waypoints = Eigen::MatrixXd(2, len);

	for (auto i = 0; i < len; ++i) {
		waypoints(0, i) = cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
		waypoints(1, i) = -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);
	}

	return waypoints;
	...

Finally my cost paraters were tuned by hand. I used an improved console printout to see more clearly the results of the MPC (printing throttle and steer angle), and a lot of "cmake .. && make && ./mpc" :)

The final parmeters chosen

	...
	const double k_cte           = 2;
	const double k_epsi          = 1;
	const double k_v             = 1;
	const double k_throttle      = 5;
	const double k_steering      = 2500;
	const double k_dthrottle     = 10;
	const double k_dsteering     = 5000;
	...

Note: I can improve this project by modelling my speed controls into the MPC's cost function. However, at this time I still havent got this working well.
	
Car In the turn

![Alt text](https://raw.githubusercontent.com/dyh1213/CarND-Model-Predictive-Control-MPC-/master/image/priorToCurve.JPG)

Car Exiting Turn

![Alt text](https://raw.githubusercontent.com/dyh1213/CarND-Model-Predictive-Control-MPC-/master/image/afterTheCurve.JPG)

My parameters were manually tuned. I found that adding a much higher cost to the angle parameter epsi is of much higher importance than the CTE (cross-track-error).

---

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
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
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
