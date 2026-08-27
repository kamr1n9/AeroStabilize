%% AeroStabilize - Roll Torque Test

clear;
clc;

addpath('../models');

% Load parameters
run('../parameters/quadcopter_params.m');

% Initial state
x0 = zeros(12,1);

% Hover thrust
T_hover = m * g;

% Apply small roll torque
tau_phi = 0.02;

% Inputs
u = [T_hover; tau_phi; 0; 0];

% Evaluate dynamics
xdot = quad_dynamics(x0, u, m, g, I);

% Display result
disp('State derivative with roll torque:')
disp(xdot)