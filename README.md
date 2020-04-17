# EECS561-Digital-Control-Project: Discrete MPC
We will implement a discrete proportional controller (on the error defined as a function of the deviation from the center-line
of the race-track) to generate an approximate trajectory that our car will follow. We will use MATLAB’s ode45
solver to generate this trajectory. We will then use this trajectory to linearize our model and attempt to implement
a discrete MPC controller to follow this trajectory. We will solve the MPC optimization problem using MATLAB’s
quadprog.

## Usage instructions:
Run main.m
