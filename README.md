# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Project Basics
The objective of this project is to use a PID (Proportional-Integral-Derivative) Controller to drive a simulated car in a virtual track. It is the steering angle that needs to be controlled by the PID. Although, the throttle can also be controlled using a similar feedback loop, I haven't ventured into it for this project. For the most part, I have tuned the P, I and D parameters manually for  steering.

Here is a [video](/result/output.mp4) of the car driving a complete lap around the track.

## Tuning the PID
I was able to get the parameters close enough to keep the car on track by manual tuning. Trying to implement TWIDDLE was't really successful in my attaempts. I got a decently optimized solution at PID parameters = (0.2, 0, 2) on trial and error method. Then they were fine tuned to (0.15, 0.005, 2.5) evnentually.

* The P parameter is the proportionality constant with the error that the car makes with the intended path. This is the most important control parameter in the PID controller. The value of P controller decides how fast or slow the control approaches the true value. But higher value of P also results in the contol overshooting the desired value. When I started tuning with parameters (0.2,0,0) the car tried to keep in lane though overshooting to  left and right and finally getting off the curb after a while.

* The D (differential) parameter kind of acts like a braking force on the the controller tries to overshoot the true value. To reduce the overshooting, I started increasing the D parameter. On reaching 2, ie parameters (0.2, 0, 2), the car was able to keep inside the  track successfully. At this point the car is driving well enough without getting outside the lanes.

* The I parameter accounts to correct any bias in the control provided by the PD alone. A good example of this one was discussed by Sebastian, where he had introduced a drift value for the steering but the controller still got the car to the true position. However, for this project I didn't find the I parameter to contribute much. I still kept a very low value for I to act if there is a drift that I couldn't notice from the simulator.

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


