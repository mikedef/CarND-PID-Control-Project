# CarND-Controls-PID 

## Michael DeFilippo

#### Please see my [project code](https://github.com/mikedef/CarND-PID-Control-Project) for any questions regarding implementation.
---

# Overview
This repository contains all the code needed to complete the final project for the PID course in Udacity's Self-Driving Car Nanodegree.

---
## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric
 - Implement a PID controller for steering
 - Optimize ini parameters for each PID coefficient
 - No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

---
## PID Controller
The PID controller is a closed loop feedback controller that calculates the output of the controller based on the difference between the current state of the feedback and the desired state. The actual implementation of the controller is straightforward. Tuning the parameters for the controller is the time consuming part. Understanding what each component of the PID controller does is important for tuning the parameters. 
 - Proportional control - 
