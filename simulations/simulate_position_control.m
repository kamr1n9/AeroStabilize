%% AeroStabilize - Cascaded Position Control

clear;
clc;

addpath('../models');

% Load parameters
run('../parameters/quadcopter_params.m');

% Initial state
x0 = zeros(12,1);

% Desired position
x_des = 3.0;
y_des = 2.0;
z_des = 2.0;

% Desired yaw
psi_des = 0;

% Position controller gains
Kp_x = 0.6;
Kd_x = 1.0;

Kp_y = 0.6;
Kd_y = 1.0;

Kp_z = 4.0;
Kd_z = 3.0;

% Attitude controller gains
Kp_phi   = 0.8;
Kd_phi   = 0.25;

Kp_theta = 0.8;
Kd_theta = 0.25;

Kp_psi   = 0.5;
Kd_psi   = 0.20;

% Simulation time
tspan = [0 12];

% Closed-loop dynamics
dynamics = @(t,x) position_closed_loop( ...
    t, x, ...
    x_des, y_des, z_des, psi_des, ...
    Kp_x, Kd_x, ...
    Kp_y, Kd_y, ...
    Kp_z, Kd_z, ...
    Kp_phi, Kd_phi, ...
    Kp_theta, Kd_theta, ...
    Kp_psi, Kd_psi, ...
    m, g, I);

% Run simulation
[t, x] = ode45(dynamics, tspan, x0);

% Plot positions
figure;

plot(t, x(:,1), 'LineWidth', 1.5);
hold on;
plot(t, x(:,2), 'LineWidth', 1.5);
plot(t, x(:,3), 'LineWidth', 1.5);

yline(x_des, '--');
yline(y_des, '--');
yline(z_des, '--');

grid on;

xlabel('Time [s]');
ylabel('Position [m]');
title('Cascaded 3D Position Control');

legend('x', 'y', 'z', ...
       'Desired x', ...
       'Desired y', ...
       'Desired z');

function xdot = position_closed_loop( ...
    t, x, ...
    x_des, y_des, z_des, psi_des, ...
    Kp_x, Kd_x, ...
    Kp_y, Kd_y, ...
    Kp_z, Kd_z, ...
    Kp_phi, Kd_phi, ...
    Kp_theta, Kd_theta, ...
    Kp_psi, Kd_psi, ...
    m, g, I)

% Current position
x_pos = x(1);
y_pos = x(2);
z_pos = x(3);

% Current velocity
vx = x(4);
vy = x(5);
vz = x(6);

% Current attitude
phi   = x(7);
theta = x(8);
psi   = x(9);

% Current angular rates
p = x(10);
q = x(11);
r = x(12);

%% Position controller

e_x = x_des - x_pos;
e_y = y_des - y_pos;
e_z = z_des - z_pos;

% Desired horizontal accelerations
ax_des = Kp_x * e_x - Kd_x * vx;
ay_des = Kp_y * e_y - Kd_y * vy;

% Convert desired acceleration into attitude commands
theta_des = ax_des / g;
phi_des   = -ay_des / g;

% Limit commanded angles
max_angle = deg2rad(20);

theta_des = max(min(theta_des, max_angle), -max_angle);
phi_des   = max(min(phi_des, max_angle), -max_angle);

%% Altitude controller

T = m*g + Kp_z*e_z - Kd_z*vz;

%% Attitude controller

e_phi   = phi_des   - phi;
e_theta = theta_des - theta;
e_psi   = psi_des   - psi;

tau_phi   = Kp_phi   * e_phi   - Kd_phi   * p;
tau_theta = Kp_theta * e_theta - Kd_theta * q;
tau_psi   = Kp_psi   * e_psi   - Kd_psi   * r;

%% Control input

u = [T;
     tau_phi;
     tau_theta;
     tau_psi];

%% Quadcopter dynamics

xdot = quad_dynamics(x, u, m, g, I);

end