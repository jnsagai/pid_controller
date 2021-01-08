# PID Controller
Self-Driving Car Engineer Nanodegree Program

In this project, a PID controller is implemented using C++ to maneuver the Udacity simulator's vehicle around the track.
The Udacity simulator provides the cross track error (CTE) and the velocity (mph). The appropriate steering angle for the vehicle is calculated using an optimized PID controller where each control parameter is fine-tuned using a method called Gradient Descent or Twiddle.

## Rubric Points

The PID Controller
----

The main goal of this project is to implement a PID (Proportional-Integral-Derivative) Controller in order to try to minimize the CTE ( cross track error ), which represent the error between the current position of the vehicle and the target position, where in this case is represented by the center of the road in the simulator.

The PID controller is based on a closed-loop system and it includes a feedback control system. This system evaluates the feedback variable using a fixed point to generate an error signal( in our case, the error signal is the CTE ). Based on that, it alters the system output. This procedure will continue till the error reaches zero otherwise the value of the feedback variable becomes equivalent to a fixed point. The image below represents a typical implementation of a PID controller:

![alt text](https://www.controleng.com/wp-content/uploads/sites/2/2014/08/CTL1408_WEB_F1_PID-Valin_Fig1_PID-control_loopslider.jpg)

The Proportional gain (P) in this case is responsible for steer the vehicle proportionally in the opposite direction of the CTE, in other words, the higher is the CTE, the higher will be the P component with the opposite signal. However, if we implement only the P controller the vehicle will tend to bounce around the target track and generate an overshooting, as can be seen in this [demonstration](https://github.com/jnsagai/pid_controller/blob/master/videos/p_controller.mp4).

The overshooting can be solved by applying another component, the Derivative gain (D). The derivative component tends to minimize the overshooting caused by the Proportional component, and then the vehicle can reach the target track in a smoother way, as can be seen in this next [demonstration](https://github.com/jnsagai/pid_controller/blob/master/videos/pd_controller.mp4), where a PD controller was applied.

However, in some cases, there is the presence of a constant error, or bias, where a PD controller is not enough to eliminate it. The Integral gain (I) is then responsible for trying to eliminate this error. This [final demonstration](https://github.com/jnsagai/pid_controller/blob/master/videos/pid_controller.mp4) shows how a complete PID controller was applied in the simulation.

Tuning the PID Controller
----

The main question regarding a PID controller is how to define the optimal value for each component. There are several methods used for that, like Manual Tuning, Gradient Descent (twiddle), or SGD (Stochastic Gradient Descent).
In the specific case of this project, the first attempt was to manually tune each parameter on a try and error approach. The first step was to find a suitable value for the Proportional component, and it was not an easy task since the vehicle kept scaping from the lane. After a plausible gain was obtained, the next step was to find the Derivative gain, in order to reduce the overshooting presented by the P controller, and finally, I worked on the Integral component, which was easily tuned since the bias on the vehicle was almost inexistent. I started using the same values of the Udacity PID Lesson then I started fine tunning then until I reached the final values of Kp = 0.215, Kd = 3.521, and Ki = 0.0042.

After I reached this goal, I started working on the implementation of the Twiddle algorithm, or Gradient Descent, as described in the PID Lessons. I implemented it using an FSM (Finite-State Machine) as my basic structure. After initializing the FSM and get the first error, the states transitioned between incrementing and decrementing the hyperparameters (or gains) until it reaches a minimum tolerance, defined a priori. After running this algorithm for some laps, I reached the final values of Kp = 0.1251, Kd = 8.2974, and Ki = 0.0013.

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
