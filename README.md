---
layout: default
---

Project Reflections
-------------------

Author: Roman Stanchak

Course: Self-Driving Car Engineer Nanodegree Program Term 2

Date: Sept, 17 2017

## The Model

*Student describes their model in detail. This includes the state, actuators and update equations.*

The model is a straightforward port of the MPC quiz.

The state consists of

- x - the x position in vehicle coordinates
- y - the y position in vehicle coordinates
- psi - the orientation in vehicle coordinates
- v - the velocity in the direction of the orientation
- cte - the cross-track error
- epsi - the orientation error 

The acutators consist of

- a - throttle control value in the interval [-1, 1]
- delta - steering control in the interval [-1, 1]

The update equations are as follows:

- x

## Timestep Length and Elapsed Duration (N & dt)

*Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.*

The lecture slides suggested minimizing dt and maximizing N as practical.  I chose dt=100ms to match the actuator latency. I'm not sure if this 

## Polynomial Fitting and MPC Preprocessing

*A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.*

The waypoints are transformed to vehicle coordinates, and they are subsequently fit to a 3rd degree polynomial f.  The waypoints are transformed in order to simplify computation of cte and epsi.  In vehicle coordinates, x, y, psi are zero, so the cte and epsi computations have the same form as the straight-line model:

- cte = y-f(x)
- epsi = 

## Model Predictive Control with Latency

*The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.* 
The implementation deals with latency as follows:

1. Cost function prioritizes smooth actuator controls -- this has the effect of averaging change in control through time, so control changes will be smaller for each time step.   A smaller change in control means the error introduced by delaying that control is less than what it would be if the magnitude of the control was larger.
2. Cost function considers Low cte and epsi error -- this has the effect of keeping overall error low, so error introduced by latency does not result in excessive overall error (and the car drives off the road).
3. error to reference velocity of 30 mph -- keeping speeds low keeps the change in position and orientation due to latency within a safe bound.
4. 100 ms time step in the optimizer -- using a 100 ms time step in the MPC implmenentation forces the optimizer to predict future states and compensatory controls at intervals matching the latency.  The intent is to insure the controls are safe.

