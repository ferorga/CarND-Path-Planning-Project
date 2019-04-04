# Path Planning Project - UDACITY
Self-Driving Car Engineer Nanodegree Program

# Introduction

This is one of the most challenging projects that I have had to face in the Udacity Nanodegree Program. There were many concepts to cover in a single project, starting from generating the smooth trajectory and communicating with the simulator to creating the multiple costs functions that allow the car to drive around the track safely.

One of the first and gratest challenge to me was to create the smooth trajectory withot doing what Aaron and David explained in their video. I found that their code was not really clean nor efficient to use it for cost evaluation. Although the video was really useful to get in touch with the simulator behaviour, I had to spend many hours doing tests myself to really understand what "previous path" was and how the simulator must to be fed with the waypoints.

Moreover, the lessons before the project were too theoretical and not really usefull to complete the project. Actually, I only used the JMT trajectory generation and the explanetion from the costs lessons. The A* and other videos were not useful to accomplish with the project requirements.

# Disclaimer

*  The current code is able to run with the simulator term 3 v1.2.
*  It can still have some collisions when overtaking.
*  It only takes into account the contiguous lanes from the current position of the car. This means that if the car is driving in one of the side lanes, it will not generate a trajectory to the opposite side lane. This can create a situation that the car is stuck in traffic even when the furthest lane is empty of cars.
*  There is still work to do :).

# Into the Code

As a reference and to better understand how to approach the project, I used the code that [kenshiro-o](https://github.com/kenshiro-o/CarND-Path-Planning-Project) developed. In the end, I only used a few helper functions from his code and I didn't take anything from the main path planning algorithm and state machine.

## Code Structure

### Main

Here is where the algorithm runs when new data is received from the simulator. It generates and keeps track of the trajectory that will be sent to the car simulator.

### Vehicle

This class was created to allocate the other car objects. It includes the *propagate* function that creates a estimated trajectory based on a basic constant velocity model. This trajectory is used to compare with our current car trajectory and evaluate the costs.

### Costs

This file only contains a few costs functions that I created, tuned and evaluated to allow the car select the best and safest trajectory possible.

### Trajectory

The Trajectory class stores the information of the points sent to the simulator in cartesian coordinate frame as well as frenet frame. This values are used in future iterations to allow continuity in the generation of the next points.

This class also includes a static function to generate a JMT (Jerk Minimizing Trajectory) given the start and end points, and the trajectory elapsed time.

### Map

TBD

### Spline

TBD

## Trajectory

TBD

## Costs

TBD
