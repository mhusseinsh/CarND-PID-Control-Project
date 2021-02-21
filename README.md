[//]: # (Image References)

[simulator]: ./output_images/simulator.png "Simulator"
[simulation]: ./output_images/simulation.gif "Simulation"
[flowchart]: ./output_images/PID_en.svg.png "Flowchart"
[p-control]: ./output_images/p.gif "p-control"
[d-control]: ./output_images/d.gif "d-control"
[i-control]: ./output_images/i.gif "i-control"
[pid-control]: ./output_images/pid.gif "pid-control"
[plot]: ./output_images/pid_plot.png "plot"
[tuning]: ./output_images/tuning.gif "tuning"

# **PID Control** 

## Report

---

**PID Control Project**
# Overview
This repository contains all the code needed to run the project for the PID Control course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
In this project the goal is to implement a PID controller to control and maneuver the car around the track in the simulator. The simulator sends data which are cross-track error (CTE), speed (mph) and angle to the PID controller using WebSocket. This data is used to calculate the appropriate steering angle and the throttle value which are sent back by the controller to drive the car. The PID uses the uWebSockets WebSocket implementation.

## Prerequisites

This project involves the Term2 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

## Running the Code
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.
```sh
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
```
Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:
```sh
1. ./clean.sh
2. ./build.sh
3. ./run.sh
```
## Connecting with the simulator
To run the PID Control after building and installing: either `./pid` or [`./run.sh`](https://github.com/mhusseinsh/CarND-PID-Control-Project/blob/master/run.sh). The output will be as below:
```sh
Listening to port 4567
```
Here the PID Control will be waiting for the simulator to launch and start, and once it started, the below message will appear in the console.
```sh
Connected!!!
```
Initially, the simulator looks like this

![alt text][simulator]

# Implementing the PID Control
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|___src
    |   json.hpp
    |   main.cpp
    |   PID.cpp
    |   PID.h
```
## PID Control
The PID Control consists of three main hyperparameters which are the P, or “proportional” component, the D, or "derivative" component and the I, or "integral" component.

These 3 parameters need to be tuned in order to better control your control value which is in this case, the steering angle and the throttle value.

Tuning the parameters can be done in different ways. Famous methods such as SGD, [Ziegler–Nichols](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method), [Twiddle](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjkps-JkfvuAhUK7KQKHXmhAI0QFjAAegQIBBAD&url=https%3A%2F%2Fmartin-thoma.com%2Ftwiddle%2F&usg=AOvVaw2cpdOH_AP0ljxoddloRe9K) or [manual tuning](https://en.wikipedia.org/wiki/PID_controller#Manual_tuning) can be applied. The goal of all the methods is to find the _K<sub>p</sub>_, _K<sub>i</sub>_ and _K<sub>d</sub>_ values.

![alt text][flowchart]

The below [table](https://en.wikipedia.org/wiki/PID_controller) summarizes the effect of each parameter independently

| **Parameter** | **Rise time** | **Overshoot** | **Settling time**  | **Steady-state** **error**  | **Stability** |
|--|--|--|--|--|--|
| _**K<sub>p</sub>**_  |  Decrease   |  Increase   |  Small change    |  Decrease   |  Degrade   |
| _**K<sub>i</sub>**_  |  Decrease   |  Increase   |  Increase    |  Eliminate   |  Degrade   |
| _**K<sub>d</sub>**_  |  Minor change   |  Decrease   |  Decrease    |  No effect in theory   |  Improve if _K<sub>d</sub>_ small   |

### **Proportional (P) component**
The P component controls the error proportionally along time. By increasing the proportional gain, the control signal is increased proportionally as well. This component has the most obvious effect on the car's behavior. It causes the vehicle to steer proportionally with respect to the CTE (the center from the lane). As an example, if the car is far to the left, so the distance is far from the center, it steers hard to the right. The problem of setting only the P value, lets the vehicle to overshoot the central line and can go out of the road quickly. An example video of only using P-controller is shown below:

![alt text][p-control]

### **Derivative (D) component**
The D component controls the rate of change of the error. Which means, if this component is used alongside with the P component, it will help to reduce the tendency of the vehicle to overshoot the center line, and makes the car moves smoorhly around the center. An example video of only using P-controller is shown below:

![alt text][d-control]


### **Integral (I) component**
The I component controls the accumalation of the error. As described in the table above, the I component tends to reduce the steady state error. This means, when adding it along with a P-D controller, it accumalates the error, thus eliminating the bias of the controlled system. If the I component is used alone, it will cause the vehicle just to drive in circles as it produces quick responses.An example video of only using P-controller is shown below:

![alt text][i-control]

The plot below shows an illustration of the combination of the three components in a single controller.

![alt text][plot]

## Tuning
[Manual Tuning](https://en.wikipedia.org/wiki/PID_controller#Manual_tuning) method is used in this project to find the optimal values of the PID gains.

The steps recommended to perform the manual tuning are:

1. Setting _K<sub>i</sub>_ and _K<sub>d</sub>_ values to zero.
2. Increasing _K<sub>p</sub>_ until the output of the loop oscillates, and reaches a steady oscillation around the desired trajectory.
3. Increasing _K<sub>i</sub>_ slightly until any offset is corrected in sufficient time for the process and the system reaches a disred trajectory.
4. Finally, increasing _K<sub>d</sub>_ until the oscillation start to disappear.

An illustration of the effect of each parameter is shown below

![alt text][tuning]

The parameters was initially chosen as _K<sub>p</sub>_, _K<sub>i</sub>_ and _K<sub>d</sub>_ equal to _0.1_, _0.0_, and _0.0_ respectively. Then the _K<sub>p</sub>_ was increased until it reached a value of a value of _0.15_. Afterwards the _K<sub>i</sub>_ was introduced. Although the _K<sub>i</sub>_ can gives quicker responses, but having a high value of it, can introduce more oscillations, so an optimal value was found that was well performing of _0.0001_. Then the _K<sub>d</sub>_ was introduced and increased until the oscillations decreased and the required trajectory was reached, and it was set to a value of _1.5_.

Moreover, a throttle PID was implemented to control the acceleration and deceleration based on the error difference between the target speed of _30.0 mph_ and the current speed, also it was tuned manually until getting optimal values.

The final values of the two PID Controllers are:
| | **_K<sub>p</sub>_** | **_K<sub>i</sub>_** | **_K<sub>d</sub>_** |
|--|--|--|--|
|**steering_pid**|_0.15_|_0.0001_|_1.5_|
|**speed_pid**|_0.1_|_0.001_|_0.1_|

It was made sure while returning the total error value, that it always lies between _[-1.0, 1.0]_.

## Results Evaluation and Success Criteria
Based on the defined [Rubric Points](https://review.udacity.com/#!/rubrics/1972/view) the PID Control should achieve the below points:

1. **Describe the effect each of the P, I, D components had in your implementation.**: This was already explained in a previous section called "**PID Control**" along with visualizations of the effect of each parameter separetly.
   
2. **Describe how the final hyperparameters were chosen.**: This was explained in a previous section called "**Tuning**" where the steps of choosing the parameters and tuning them was explained.
   
3. **The vehicle must successfully drive a lap around the track.**: The vehicle was able to drive in only the drivable portion of the track surface, while not popping up onto the ledges or rolling over any surface, also its steering was quite smooth. Final results of the PID Controller is shown below:
  ![alt text][pid-control]