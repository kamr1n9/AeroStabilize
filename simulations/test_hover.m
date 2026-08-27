%% AeroStabilize - Hover Test

clear;
clc;

addpath('../models');

% Load parameters
run('../parameters/quadcopter_params.m');

% Initial state
x0 = zeros(12,1);

% Hover thrust
T_hover = 1.2 * m * g;

% No rotational torques
u = [T_hover; 0; 0; 0];

% Evaluate dynamics
xdot = quad_dynamics(x0, u, m, g, I);

% Display result
disp('State derivative at hover:')
disp(xdot)