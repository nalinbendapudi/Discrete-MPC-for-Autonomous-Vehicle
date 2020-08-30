# Trajectory Optimization using Discrete MPC
In this implementation, we attempt to control a car using trajectory optimization techniques. The car is assumed to follow the dynamics of a bicycle model. To achieve stable and safe tracking, we are using discrete-time Model Predictive Control.

A discrete proportional controller (on the error defined as a function of the deviation from the center-line of the race-track) is used to generate an approximate trajectory that our car will follow. We will use MATLAB’s `ode45` solver to generate this trajectory. We then use this trajectory to linearize our model and attempt to implement a discrete MPC controller to follow this trajectory. We solved the MPC optimization problem using MATLAB’s `quadprog`.

## Report
For more details on the technique and implementation, read our report: https://github.com/nalinbendapudi/Discrete-MPC-for-Autonomous-Vehicle/blob/master/Digital_Control_Project_Report.pdf

## Usage instructions:
After you clone or download this repository, open MATLAB on your system and run `main.m`
