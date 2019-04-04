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
*  My approach does not use a state machine. I found easier to create several paths from the car and evaluate all of them agains the cost functions.
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

This class includes some methods to convert from cartesian to frenet and smooth the path by using a spline function between all the waypoints given in the csv file.

### Spline

[External library](https://kluge.in-chemnitz.de/opensource/spline/) used to create spline functions and smooth the trajectory.

## Trajectory

A trajectory is a set (vector) of points in cartesian coordinates that is sent to the simulator to make it move. The simulator "eats" one point of the array every 0.02s (50 times per second). Then, depending on how far we set the points to each other, we will be given the velocity of the car. 

My code includes a definition of the *time horizon*. This parameter will then set the number of points of our trajectory and how long it is depending on the velocity. By default it is set to 2, meaning that the whole trajectory will last 2 seconds if we only send it one time. 

Then, in each iteration, we check how many points the simulator has "eaten" and then we will add the same number of points to our next trajectory. It is important to understand that we need to send again the whole trajectory, not only the new points. This gave me some headaches because the simulator stores the "previous path" but it is required to send it again if you want to keep the continuity of the movements.

The new trajectory is created from a certain point of the older trajectory (30 by default). For example, if our trajectory has 100 points by default, what we do is to keep up to the 30th point of our trajectory and we generate 70 points more with different target S and D. I decided to create the following trajectories from this 30th point:

* 50 Trajectories ahead the car with a gaussian generator that will modify the target S and D.
* 50 Trajectories ahead the car in the right lane to the car with a gaussian generator that will modify the target S and D.
* 50 Trajectories ahead the car in the left lane to the car with a gaussian generator tat will modify the target S and D.

The problem with this approach is that if the car is in the very left or right lane wi will not have trajectories created to the opposite lane.

The D gaussian generator has a standard deviation defined by default to be 0.2.
The S gaussian generator has a variable standard deviation that will depend on the older S and velocity.

## Vehicles

I only keep track of all the cars 70 meters ahead and 30 meters behind our car and in our direction. Then I propagate their trajectory the same amount of time of our car trajectory time horizon. The other cars and our car must have the same trajectory length.

## Costs

The cost functions were tuned experimentally. Although I used desmos to easily see the limits and function parameters.
The current code is not using of all them. I created some that didn't give me the expected results but I decided to keep them in the code for future use or improvement.
