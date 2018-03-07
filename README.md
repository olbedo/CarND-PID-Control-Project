# PID Controller Project
**from Udacity's Self-Driving Car Engineer Nanodegree Program**

## Introduction
The aim of this project is to build a PID controller which drives a vehicle in a simulator. The hyperparameters of the controller shall be tuned as described in the lessons until the vehicle successfully completes at least one lap around the track. The vehicle should drive as fast as possible but still savely.

## Rubric Points
### Implementation
The PID controller was implemented in C++ in accordance with the algorithm explained in the lessons. For hyperparameter tuning I wrote an additional class named `Twiddle`.  

### Hyperparameter Tuning
In order to tune the hyperparameters of the PID controller it is essential to find suitable values for the initialization. Otherwise the vehicle veers off the track quickly.
As my mentor recommended, I used the [Ziegler–Nichols method](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method) to find proper hyperparameters. I reduced the throttle to 0.2 as the initial value of 0.3 caused the vehicle to get stuck at the curb before reaching the target velocity.

For the determination of the ultimate gain $K_u$ and the oscillation period $T_u$ I set the coefficients $K_d$ and $K_i$ to zero and just altered $K_p$ until I reached a stable oscillation. In the first step I set $K_p$ to 0.1 and ran the simulator while recording the cross-track error (CTE). Then I increased $K_p$ by 0.1 and started the simulator again. I repeated this procedure until $K_p=0.8$.

Afterwards I analysed the recorded data. The first 1000 steps I disregarded since the vehicle has not reached the nominal velocity yet. The most stable oscillation I got with $K_p=0.4$. Thus, the ultimate gain $K_u$ is 0.4. The measured oscillation period $T_u$ is 235. Inserting these values into the formulae of the [Ziegler–Nichols method](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method) I got

\[
K_p=0.6 \cdot K_u=0.24
\]

\[
T_i=T_u/2=117.5
\]

\[
K_i=K_p/T_i=0.002
\]

\[
T_d=T_u/8=29.4
\]

\[
K_d=K_p\cdot T_d=7.1
\]

With these hyperparameters the PID controller steers the vehicle safely around the track while the throttle is set to 0.2. For higher velocities the parameters are not optimal because the dynamics of the system changes. However, the values are a good starting point for  the hyperparameter tuning.

I used another PID controller to control the speed based on the absolute value of th current steering value. In fact it is just a P controller since I only used the proportional part of the controller by setting the coefficients $K_d$ and $K_i$ to zero. After some experimentations I found the P coefficient $K_p=2.0$ and a maximum throttle value of 0.75 a good starting set for the following hyperparameter tuning.

I initialized `Twiddle` with the coefficients mentioned above and run the simulator. After every 4500 steps (which equals roughly on lap) the twiddle algorithm updates one coefficient according to the corresponding increment. As a measure for the performance of the hyperparameters I used the highest absolute value of the CTE within this interval. This turned out to work better than the mean squarred error. In the table below the results of the parameter tuning is shown.

| lap | $K_{p,steer}$ | $K_{p,steer}$ | $K_{d,steer}$ | $K_{p,throttle}$ | max error |
|:-:|--:|--:|--:|--:|--:|
| 1 | 0.24 | 0.002 | 7.1 | 2 | 2.8701 |
| 2 | 0.25 | 0.002 | 7.1 | 2 | 2.8701 |
| 3 | 0.23 | 0.002 | 7.1 | 2 | **2.7877** |
| 4 | 0.23 | 0.0021 | 7.1 | 2 | **2.7829** |
| 5 | 0.23 | 0.0021 | 7.6 | 2 | 3.2571 |
| 6 | 0.23 | 0.0021 | 6.6 | 2 | **2.6802** |
| 7 | 0.23 | 0.0021 | 6.6 | 2.1 | 2.8903 |
| 8 | 0.23 | 0.0021 | 6.6 | 1.9 | **2.6339** |
| 9 | 0.241 | 0.0021 | 6.6 | 1.9 | 2.7123 |
| 10 | 0.219 | 0.0021 | 6.6 | 1.9 | 2.7197 |
| 11 | 0.23 | 0.00221 | 6.6 | 1.9 | 2.7439 |
| 12 | 0.23 | 0.00199 | 6.6 | 1.9 | **2.4871** |
| 13 | 0.23 | 0.00199 | 7.15 | 1.9 | 3.2099 |
| 14 | 0.23 | 0.00199 | 6.05 | 1.9 | **2.419** |
| 15 | 0.23 | 0.00199 | 6.05 | 2.01 | 2.7823 |
| 16 | 0.23 | 0.00199 | 6.05 | 1.79 | 2.7231 |
| 17 | 0.2399 | 0.00199 | 6.05 | 1.9 | 2.7437 |
| 18 | 0.2201 | 0.00199 | 6.05 | 1.9 | 2.6254 |
| 19 | 0.23 | 0.002111 | 6.05 | 1.9 | 2.5601 |
| 20 | 0.23 | 0.001869 | 6.05 | 1.9 | 3.0947 |
| 21 | 0.23 | 0.00199 | 6.655 | 1.9 | 2.5832 |
| 22 | 0.23 | 0.00199 | 5.445 | 1.9 | 2.7944 |
| 23 | 0.23 | 0.00199 | 6.05 | 1.999 | 2.7679 |
| 24 | 0.23 | 0.00199 | 6.05 | 1.801 | 2.7566 |

Finally, I chose the hyperparameters where the highest absolute value of the CTE in one lap is minimal, that is:
\[
K_{p,steer}=0.23, \quad K_{i,steer}=0.002, \quad K_{d,steer}=6.1
\]
\[
K_{p,throttle}=1.9, \quad K_{i,throttle}=0.0, \quad K_{d,throttle}=0.0
\]
With these coefficients the vehicle reached a maximum speed as high as 62.

### Effect of the P, I, D components in the implementation

The P component of the PID controller is the proportional part, i.e. the control output is proportional to the current error but in opposite direction. However, it does not take the current steering angle into consideration which already acts against the error. To avoid or at least minimize overshooting the D (differential) component account for this difference in the error. The I component is the integral part of the controller. It corrects for the bias of the steering. 

---

The lines below are from the original README file of the [Udacity CarND-Controls-PID project](https://github.com/udacity/CarND-PID-Control-Project)

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
