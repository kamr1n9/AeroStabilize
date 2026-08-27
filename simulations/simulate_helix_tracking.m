%% AeroStabilize - Helix Trajectory Tracking

clear;
clc;

addpath('../models');
addpath('../trajectories');

% Load parameters
run('../parameters/quadcopter_params.m');

%% Initial state

x0 = zeros(12,1);

% Start at the beginning of the helix
[x_start, y_start, z_start, ~, ~, ~] = helix_trajectory(0);

x0(1) = x_start;
x0(2) = y_start;
x0(3) = z_start;

%% Desired yaw

psi_des = 0;

%% Position controller gains

Kp_x = 1.0;
Kd_x = 1.4;

Kp_y = 1.0;
Kd_y = 1.4;

Kp_z = 4.0;
Kd_z = 3.0;

%% Attitude controller gains

Kp_phi = 0.8;
Kd_phi = 0.25;

Kp_theta = 0.8;
Kd_theta = 0.25;

Kp_psi = 0.5;
Kd_psi = 0.20;

%% Simulation time

tspan = [0 20];

%% Closed-loop dynamics

dynamics = @(t,x) helix_closed_loop( ...
    t, x, ...
    psi_des, ...
    Kp_x, Kd_x, ...
    Kp_y, Kd_y, ...
    Kp_z, Kd_z, ...
    Kp_phi, Kd_phi, ...
    Kp_theta, Kd_theta, ...
    Kp_psi, Kd_psi, ...
    m, g, I);

%% Run simulation

[t, x] = ode45(dynamics, tspan, x0);

%% Generate desired trajectory for plotting

x_des_plot = zeros(size(t));
y_des_plot = zeros(size(t));
z_des_plot = zeros(size(t));

for k = 1:length(t)

    [x_des_plot(k), ...
     y_des_plot(k), ...
     z_des_plot(k), ...
     ~, ~, ~] = helix_trajectory(t(k));

end

%% Tracking error

error_x = x_des_plot - x(:,1);
error_y = y_des_plot - x(:,2);
error_z = z_des_plot - x(:,3);

position_error = sqrt( ...
    error_x.^2 + ...
    error_y.^2 + ...
    error_z.^2);

% RMSE
RMSE = sqrt(mean(position_error.^2));

% Maximum tracking error
max_error = max(position_error);

fprintf('Trajectory RMSE: %.4f m\n', RMSE);
fprintf('Maximum Tracking Error: %.4f m\n', max_error);

%% 3D trajectory plot

figure;

plot3( ...
    x_des_plot, ...
    y_des_plot, ...
    z_des_plot, ...
    '--', ...
    'LineWidth', 1.5);

hold on;

plot3( ...
    x(:,1), ...
    x(:,2), ...
    x(:,3), ...
    'LineWidth', 1.8);

grid on;

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

title('AeroStabilize - Helix Trajectory Tracking');

legend( ...
    'Desired Trajectory', ...
    'Actual Trajectory');

view(3);

%% Position error plot

figure;

plot( ...
    t, ...
    position_error, ...
    'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Position Error [m]');
title('Helix Trajectory Tracking Error');

%% Closed-loop controller function

function xdot = helix_closed_loop( ...
    t, x, ...
    psi_des, ...
    Kp_x, Kd_x, ...
    Kp_y, Kd_y, ...
    Kp_z, Kd_z, ...
    Kp_phi, Kd_phi, ...
    Kp_theta, Kd_theta, ...
    Kp_psi, Kd_psi, ...
    m, g, I)

%% Desired trajectory

[x_des, y_des, z_des, ...
 vx_des, vy_des, vz_des, ...
 ax_ff, ay_ff, az_ff] = helix_trajectory(t);

%% Current states

x_pos = x(1);
y_pos = x(2);
z_pos = x(3);

vx = x(4);
vy = x(5);
vz = x(6);

phi   = x(7);
theta = x(8);
psi   = x(9);

p = x(10);
q = x(11);
r = x(12);

%% Position errors

e_x = x_des - x_pos;
e_y = y_des - y_pos;
e_z = z_des - z_pos;

e_vx = vx_des - vx;
e_vy = vy_des - vy;
e_vz = vz_des - vz;

%% Position controller

ax_des = ax_ff + ...
    Kp_x * e_x + ...
    Kd_x * e_vx;

ay_des = ay_ff + ...
    Kp_y * e_y + ...
    Kd_y * e_vy;

%% Convert desired acceleration into attitude

theta_des = ax_des / g;
phi_des   = -ay_des / g;

%% Limit attitude commands

max_angle = deg2rad(20);

theta_des = max( ...
    min(theta_des, max_angle), ...
    -max_angle);

phi_des = max( ...
    min(phi_des, max_angle), ...
    -max_angle);

%% Altitude controller

T = m*g + ...
    Kp_z * e_z + ...
    Kd_z * e_vz;

%% Attitude controller

e_phi   = phi_des   - phi;
e_theta = theta_des - theta;
e_psi   = psi_des   - psi;

tau_phi = ...
    Kp_phi * e_phi - ...
    Kd_phi * p;

tau_theta = ...
    Kp_theta * e_theta - ...
    Kd_theta * q;

tau_psi = ...
    Kp_psi * e_psi - ...
    Kd_psi * r;

%% Control input

u = [T;
     tau_phi;
     tau_theta;
     tau_psi];

%% Dynamics

xdot = quad_dynamics(x, u, m, g, I);

end