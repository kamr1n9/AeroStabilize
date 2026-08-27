%% AeroStabilize - Roll Attitude Control

clear;
clc;

addpath('../models');

% Load parameters
run('../parameters/quadcopter_params.m');

% Initial state
x0 = zeros(12,1);

% Desired roll angle
phi_des = deg2rad(10);

% Controller gains
Kp_phi = 0.8;
Kd_phi = 0.25;

% Simulation time
tspan = [0 5];

% Closed-loop dynamics
dynamics = @(t,x) roll_closed_loop( ...
    t, x, phi_des, Kp_phi, Kd_phi, m, g, I);

% Run simulation
[t, x] = ode45(dynamics, tspan, x0);

% Convert roll angle to degrees
phi_deg = rad2deg(x(:,7));

% Plot roll response
figure;
plot(t, phi_deg, 'LineWidth', 1.5);
hold on;
yline(rad2deg(phi_des), '--', 'Desired Roll');
grid on;

xlabel('Time [s]');
ylabel('Roll Angle [deg]');
title('Closed-Loop Roll Attitude Control');
legend('Actual Roll', 'Desired Roll');

function xdot = roll_closed_loop(t, x, phi_des, Kp_phi, Kd_phi, m, g, I)

% Current roll angle
phi = x(7);

% Current roll rate
p = x(10);

% Roll angle error
e_phi = phi_des - phi;

% Desired roll rate = 0
e_p = 0 - p;

% PD roll controller
tau_phi = Kp_phi * e_phi + Kd_phi * e_p;

% Hover thrust
T = m * g;

% No pitch or yaw torque
u = [T; tau_phi; 0; 0];

% Quadcopter dynamics
xdot = quad_dynamics(x, u, m, g, I);

end