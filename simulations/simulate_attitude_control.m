%% AeroStabilize - 3-Axis Attitude Control

clear;
clc;

addpath('../models');

% Load parameters
run('../parameters/quadcopter_params.m');

% Initial state
x0 = zeros(12,1);

% Desired attitude
phi_des   = deg2rad(10);   % Roll
theta_des = deg2rad(-5);   % Pitch
psi_des   = deg2rad(30);   % Yaw

% Controller gains
Kp_phi   = 0.8;
Kd_phi   = 0.25;

Kp_theta = 0.8;
Kd_theta = 0.25;

Kp_psi   = 0.5;
Kd_psi   = 0.20;

% Simulation time
tspan = [0 6];

% Closed-loop dynamics
dynamics = @(t,x) attitude_closed_loop( ...
    t, x, ...
    phi_des, theta_des, psi_des, ...
    Kp_phi, Kd_phi, ...
    Kp_theta, Kd_theta, ...
    Kp_psi, Kd_psi, ...
    m, g, I);

% Run simulation
[t, x] = ode45(dynamics, tspan, x0);

% Convert angles to degrees
phi_deg   = rad2deg(x(:,7));
theta_deg = rad2deg(x(:,8));
psi_deg   = rad2deg(x(:,9));

% Plot attitude response
figure;

plot(t, phi_deg, 'LineWidth', 1.5);
hold on;

plot(t, theta_deg, 'LineWidth', 1.5);
plot(t, psi_deg, 'LineWidth', 1.5);

yline(rad2deg(phi_des), '--');
yline(rad2deg(theta_des), '--');
yline(rad2deg(psi_des), '--');

grid on;

xlabel('Time [s]');
ylabel('Angle [deg]');
title('3-Axis Attitude Control');

legend('Roll', 'Pitch', 'Yaw', ...
       'Desired Roll', ...
       'Desired Pitch', ...
       'Desired Yaw');

function xdot = attitude_closed_loop( ...
    t, x, ...
    phi_des, theta_des, psi_des, ...
    Kp_phi, Kd_phi, ...
    Kp_theta, Kd_theta, ...
    Kp_psi, Kd_psi, ...
    m, g, I)

% Current attitude
phi   = x(7);
theta = x(8);
psi   = x(9);

% Current angular rates
p = x(10);
q = x(11);
r = x(12);

% Attitude errors
e_phi   = phi_des   - phi;
e_theta = theta_des - theta;
e_psi   = psi_des   - psi;

% PD controllers
tau_phi = Kp_phi * e_phi - Kd_phi * p;

tau_theta = Kp_theta * e_theta - Kd_theta * q;

tau_psi = Kp_psi * e_psi - Kd_psi * r;

% Hover thrust
T = m * g;

% Control input
u = [T;
     tau_phi;
     tau_theta;
     tau_psi];

% Quadcopter dynamics
xdot = quad_dynamics(x, u, m, g, I);

end