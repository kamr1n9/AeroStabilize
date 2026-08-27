%% AeroStabilize - PD vs PID Altitude under +20% Mass Uncertainty

clear;
clc;
close all;

addpath('../models');

run('../parameters/quadcopter_params.m');

%% Mass mismatch

m_nominal = m;
m_actual  = 1.20 * m_nominal;

%% Desired altitude

z_des = 2.0;

%% Controller gains

Kp_z = 4.0;
Kd_z = 3.0;
Ki_z = 1.0;

%% Simulation time

tspan = [0 20];

%% ============================================================
%% PD SIMULATION
%% ============================================================

x0_pd = zeros(12,1);

pd_dyn = @(t,x) pd_altitude( ...
    x, ...
    z_des, ...
    Kp_z, Kd_z, ...
    m_nominal, m_actual, ...
    g, I);

[t_pd, x_pd] = ode45(pd_dyn, tspan, x0_pd);

%% ============================================================
%% PID SIMULATION
%% ============================================================

% 13th state = integral of altitude error
x0_pid = zeros(13,1);

pid_dyn = @(t,x) pid_altitude( ...
    x, ...
    z_des, ...
    Kp_z, Ki_z, Kd_z, ...
    m_nominal, m_actual, ...
    g, I);

[t_pid, x_pid] = ode45(pid_dyn, tspan, x0_pid);

%% ============================================================
%% ERRORS
%% ============================================================

pd_error = z_des - x_pd(:,3);
pid_error = z_des - x_pid(:,3);

fprintf('\n============================================\n');
fprintf('PD vs PID - +20%% MASS UNCERTAINTY\n');
fprintf('============================================\n');

fprintf('PD final altitude error:  %.4f m\n', pd_error(end));
fprintf('PID final altitude error: %.4f m\n', pid_error(end));

%% ============================================================
%% PLOT 1 - ALTITUDE
%% ============================================================

figure;

plot(t_pd, x_pd(:,3), 'LineWidth', 1.6);
hold on;

plot(t_pid, x_pid(:,3), 'LineWidth', 1.6);

yline(z_des, '--', 'Desired Altitude');

grid on;

xlabel('Time [s]');
ylabel('Altitude z [m]');

title('PD vs PID under +20% Mass Uncertainty');

legend( ...
    'PD', ...
    'PID', ...
    'Desired');

%% ============================================================
%% PLOT 2 - ALTITUDE ERROR
%% ============================================================

figure;

plot(t_pd, pd_error, 'LineWidth', 1.6);
hold on;

plot(t_pid, pid_error, 'LineWidth', 1.6);

yline(0, '--');

grid on;

xlabel('Time [s]');
ylabel('Altitude Error [m]');

title('Integral Action Eliminates Steady-State Error');

legend( ...
    'PD Error', ...
    'PID Error');

%% ============================================================
%% PD FUNCTION
%% ============================================================

function xdot = pd_altitude( ...
    x, ...
    z_des, ...
    Kp_z, Kd_z, ...
    m_nominal, m_actual, ...
    g, I)

z  = x(3);
vz = x(6);

e_z  = z_des - z;
e_vz = -vz;

T = ...
    m_nominal*g + ...
    Kp_z*e_z + ...
    Kd_z*e_vz;

u = [T; 0; 0; 0];

xdot = quad_dynamics( ...
    x, ...
    u, ...
    m_actual, ...
    g, ...
    I);

end

%% ============================================================
%% PID FUNCTION
%% ============================================================

function xdot_aug = pid_altitude( ...
    x_aug, ...
    z_des, ...
    Kp_z, Ki_z, Kd_z, ...
    m_nominal, m_actual, ...
    g, I)

x = x_aug(1:12);

integral_z = x_aug(13);

z  = x(3);
vz = x(6);

e_z  = z_des - z;
e_vz = -vz;

T = ...
    m_nominal*g + ...
    Kp_z*e_z + ...
    Ki_z*integral_z + ...
    Kd_z*e_vz;

u = [T; 0; 0; 0];

xdot = quad_dynamics( ...
    x, ...
    u, ...
    m_actual, ...
    g, ...
    I);

integral_z_dot = e_z;

xdot_aug = [ ...
    xdot;
    integral_z_dot];

end