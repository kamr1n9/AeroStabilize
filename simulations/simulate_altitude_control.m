%% AeroStabilize - Altitude Control

clear;
clc;

addpath('../models');

% Load parameters
run('../parameters/quadcopter_params.m');

% Initial state
x0 = zeros(12,1);

% Desired altitude
z_des = 2.0;

% Controller gains
Kp_z = 4.0;
Kd_z = 3.0;

% Simulation time
tspan = [0 8];

% Closed-loop dynamics
dynamics = @(t,x) altitude_closed_loop(t, x, z_des, Kp_z, Kd_z, m, g, I);

% Run simulation
[t, x] = ode45(dynamics, tspan, x0);

% Plot altitude
figure;
plot(t, x(:,3), 'LineWidth', 1.5);
hold on;
yline(z_des, '--', 'Desired Altitude');
grid on;

xlabel('Time [s]');
ylabel('Altitude z [m]');
title('Closed-Loop Altitude Control');
legend('Actual Altitude', 'Desired Altitude');

function xdot = altitude_closed_loop(t, x, z_des, Kp_z, Kd_z, m, g, I)

% Current altitude
z = x(3);

% Current vertical velocity
vz = x(6);

% Position error
e_z = z_des - z;

% Velocity error
e_vz = 0 - vz;

% PD controller
T = m*g + Kp_z*e_z + Kd_z*e_vz;

% No roll, pitch or yaw torque yet
u = [T; 0; 0; 0];

% Quadcopter dynamics
xdot = quad_dynamics(x, u, m, g, I);

end