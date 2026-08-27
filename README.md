# AeroStabilize

**Nonlinear 6-DOF Quadcopter Flight Control, Trajectory Tracking, and Robustness Analysis in MATLAB**

AeroStabilize is a simulation-based flight-control project for a nonlinear quadcopter model.  
The project explores classical and optimal control strategies, actuator constraints, disturbance rejection, model uncertainty, and statistical robustness.

The complete system is simulated in MATLAB using a nonlinear 12-state rigid-body model and includes controller comparisons between PD, PID, LQR, and LQI architectures.

## Key Features

- Nonlinear 6-DOF quadcopter dynamics
- 12-state rigid-body model
- Cascaded PD position and attitude control
- PID altitude control
- Anti-windup implementation
- Acceleration feedforward
- Motor control allocation and actuator saturation
- LQR full-state feedback control
- LQI with integral action in x, y, and z
- Circular, helical, and Figure-8 trajectories
- Wind disturbance rejection
- Payload and mass uncertainty testing
- LQR vs LQI robustness mission
- 100-run Monte-Carlo robustness analysis
- 3D flight visualization and controller animations
