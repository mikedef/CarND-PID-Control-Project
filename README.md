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
 - Proportional Control - The proportional controller will stear the vehicle in proportion to the cross-track error (CTE). The CTE is the distance the vehicle is from the middle of the road. As the vehicle drifts from the right side of the road the controller will pull the vehicle left towards the middle of the road. As the vehicle drifts from the left side of the road the controller will pull the vehicle right towards the middle of the road. 
 - Integral Control - The integral controller will sum up the error or CTE and turn the vehicle toward the middle of the road. The integral term will help increase the time the controller will take to reach a desired state, but will not help with the overshoot that will build up as a result of implimentation. 
 - Derivitive Control - The derivitive controller is the change in the error or CTE from one time step to the next. Using the derivitive controller will decrease the amount of overshoot in error correction. 
 
As an experiment I implemented each controller by themselves. The vehicle was not able to drive around the track until I implemented a full PID controller. I tuned the parameters by hand. The vehicle makes it around the track without going over the edges, but oscillates back and forth as the PID controller tries to correct for CTE. Please see the project video of my vehilcle going around the track [here](https://youtu.be/vbkYn6jW1gM). 

Below are some notes I took as I changed parameters and ran different experiments. 
 - P Only:
    --Kp = 1.0     Oscillate wildly after a few seconds
    --Kp = 0.5     Oscillate wildly after 5 seconds
    -Kp = 0.1     Oscillate wildly at first turn
    -Kp = 0.05    Oscillate wildly after first turn

 - I Only:
    -Ki = 0.5     Turns max angle and gets stuck there
    -Ki = 0.05    Turns max angle left and trys max angle right
    -Ki = 0.005   Turns max angle left and trys max angle right

 - D Only:
    -Kd = 1.0     Doesn't turn enough
    -Kd = 2.0     Better but still doesn't turn enough at the turns

 - PD:
    -Kp = 0.05, Kd = 1.0
      Gets around the whole track with minor edge touching
    -Kp = 0.05, Kd = 1.5
      Better but still close to the edge during turns
    -Kp = 0.05, Kd = 2.5
      Better but still close to the edge during turns

 - PID:
    -Kp = 0.05, Ki = 0.005, Kd = 3.0
      Gets around the whole track, but oscillates while driving
    -Kp = 0.15, Ki = 0.005, Kd = 3.0
      Gets around the whole track, but oscillates while driving

