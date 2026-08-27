%% AeroStabilize - Open Loop Simulation

clear;
clc;

addpath('../models');

% Load parameters
run('../parameters/quadcopter_params.m');

% Initial state
x0 = zeros(12,1);

% Simulation time
tspan = [0 5];

% Constant input: 20% more than hover thrust
T = 1.2 * m * g;
u = [T; 0; 0; 0];

% Differential equation
dynamics = @(t,x) quad_dynamics(x, u, m, g, I);

% Run simulation
[t, x] = ode45(dynamics, tspan, x0);

% Plot altitude
figure;
plot(t, x(:,3), 'LineWidth', 1.5);
grid on;

xlabel('Time [s]');
ylabel('Altitude z [m]');
title('Open-Loop Vertical Motion');