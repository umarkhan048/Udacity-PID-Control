# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

This project is a part of the Udacity's Self-Driving Car Engineer Nano-Degree. The simulator provided by Udacity is used. The simulator sends the cross-track error, velocity and angle to the PID controller. The PID controller sends steering angle to the simulator. The goal is that the vehicle must successfully drive a lap around the track.

## Control Parameters (P, I & D)

The parameter P (Proportional) is a parameter which is set in proportion to the error. In this case, the simulator sends a cross track error to the controller. The controller sets the proportional gain (Kp) according to the cross track error.

The parameter I (Integral) is a parameter which corrects for the error accumulated over time. An example of this is error in calibration of the steering wheel. When the steering wheel is exactly at zero but the actual angle of wheel does not show a value of zero, this leads to an error which persists over time and must be continuously corrected.

The parameter D (Differential) is a parameters which does not consider the error, instead, it considers the rate of change of error. If the absence of D parameter, a vehicle tracking the lateral reference will try to reduce the deviation from lateral reference, however, it will do so in a constant lateral velocity. This leads to an overshoot. This overshoot will be corrected by the P controller but it may result in an undershoot. The D parameters helps to avoid such osciallations by reducing the corrected value in proportion to the distance from the lateral reference.

## Selection of Parameters

For this project, the parameters are tuned manually. For a start, it is assumed that the steering wheel and driving wheels are perfectly aligned, therefore, there is no accumulation of error over time. For this reason, the I parameter is set to zero. With all the parameters sets to zero, the vehicle was driving perfectly straight. The value of P is chosen rather arbitrarily. For the chosen value of 0.1, the vehicle starts following the road but overshoots after driving for a short time. The value of the D parameter was chosen by trial and error to offset the overshooting behavior. The value is found to be 2.5. 


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
