%% AeroStabilize - Anti-Windup Comparison
% Compare PID altitude control:
% 1) without anti-windup
% 2) with conditional-integration anti-windup
%
% Stress test:
% Large altitude step + intentionally reduced motor thrust limit.

clear;
clc;

addpath('../models');

% Load parameters
run('../parameters/quadcopter_params.m');

%% Test configuration

% Initial altitude
z0 = 0;

% Large commanded altitude step
z_des = 5.0;      % [m]

% PID gains
Kp_z = 4.0;
Ki_z = 1.5;
Kd_z = 3.0;

% Intentionally restrictive motor limit
% Nominal hover per motor ≈ 3.68 N
% This leaves limited excess thrust.
motor_thrust_min = 0.0;
motor_thrust_max = 4.3;    % [N]

% Simulation duration
tspan = [0 20];

%% Initial augmented state

% [x y z vx vy vz phi theta psi p q r integral_z]

x0 = zeros(13,1);

x0(3) = z0;
x0(13) = 0;

%% ============================================================
%% Simulation 1: PID WITHOUT anti-windup
%% ============================================================

dyn_no_aw = @(t,x) altitude_pid_no_aw( ...
    t, x, ...
    z_des, ...
    Kp_z, Ki_z, Kd_z, ...
    m, g, I, L, k_yaw, ...
    motor_thrust_min, motor_thrust_max);

[t_no_aw, x_no_aw] = ode45( ...
    dyn_no_aw, ...
    tspan, ...
    x0);

%% ============================================================
%% Simulation 2: PID WITH anti-windup
%% ============================================================

dyn_aw = @(t,x) altitude_pid_aw( ...
    t, x, ...
    z_des, ...
    Kp_z, Ki_z, Kd_z, ...
    m, g, I, L, k_yaw, ...
    motor_thrust_min, motor_thrust_max);

[t_aw, x_aw] = ode45( ...
    dyn_aw, ...
    tspan, ...
    x0);

%% ============================================================
%% Re-evaluate controller data for logging
%% ============================================================

motor_no_aw = zeros(length(t_no_aw),4);
motor_aw = zeros(length(t_aw),4);

sat_no_aw = zeros(length(t_no_aw),1);
sat_aw = zeros(length(t_aw),1);

for k = 1:length(t_no_aw)

    [~, motor_k, sat_k] = altitude_pid_no_aw( ...
        t_no_aw(k), ...
        x_no_aw(k,:)', ...
        z_des, ...
        Kp_z, Ki_z, Kd_z, ...
        m, g, I, L, k_yaw, ...
        motor_thrust_min, motor_thrust_max);

    motor_no_aw(k,:) = motor_k';
    sat_no_aw(k) = sat_k;

end

for k = 1:length(t_aw)

    [~, motor_k, sat_k] = altitude_pid_aw( ...
        t_aw(k), ...
        x_aw(k,:)', ...
        z_des, ...
        Kp_z, Ki_z, Kd_z, ...
        m, g, I, L, k_yaw, ...
        motor_thrust_min, motor_thrust_max);

    motor_aw(k,:) = motor_k';
    sat_aw(k) = sat_k;

end

%% ============================================================
%% Performance metrics
%% ============================================================

z_no_aw = x_no_aw(:,3);
z_aw = x_aw(:,3);

error_no_aw = z_des - z_no_aw;
error_aw = z_des - z_aw;

%% Overshoot

overshoot_no_aw = max( ...
    0, ...
    max(z_no_aw) - z_des);

overshoot_aw = max( ...
    0, ...
    max(z_aw) - z_des);

%% Maximum altitude

max_altitude_no_aw = max(z_no_aw);
max_altitude_aw = max(z_aw);

%% Final error

final_error_no_aw = abs(error_no_aw(end));
final_error_aw = abs(error_aw(end));

%% Settling time
% Defined as entering and remaining inside +/- 0.10 m

settling_band = 0.10;

settling_no_aw = calculate_settling_time( ...
    t_no_aw, ...
    error_no_aw, ...
    settling_band);

settling_aw = calculate_settling_time( ...
    t_aw, ...
    error_aw, ...
    settling_band);

%% Saturation percentage

sat_percentage_no_aw = ...
    100 * mean(sat_no_aw);

sat_percentage_aw = ...
    100 * mean(sat_aw);

%% Print results

fprintf('\n============================================\n');
fprintf('ANTI-WINDUP COMPARISON\n');
fprintf('============================================\n');

fprintf('\nWITHOUT Anti-Windup\n');
fprintf('Maximum altitude: %.3f m\n', ...
    max_altitude_no_aw);
fprintf('Overshoot: %.3f m\n', ...
    overshoot_no_aw);
fprintf('Final altitude error: %.4f m\n', ...
    final_error_no_aw);
fprintf('Settling time: %.3f s\n', ...
    settling_no_aw);
fprintf('Time in saturation: %.2f %%\n', ...
    sat_percentage_no_aw);

fprintf('\nWITH Anti-Windup\n');
fprintf('Maximum altitude: %.3f m\n', ...
    max_altitude_aw);
fprintf('Overshoot: %.3f m\n', ...
    overshoot_aw);
fprintf('Final altitude error: %.4f m\n', ...
    final_error_aw);
fprintf('Settling time: %.3f s\n', ...
    settling_aw);
fprintf('Time in saturation: %.2f %%\n', ...
    sat_percentage_aw);

%% ============================================================
%% Plot 1: Altitude comparison
%% ============================================================

figure;

plot( ...
    t_no_aw, ...
    z_no_aw, ...
    'LineWidth', 1.5);

hold on;

plot( ...
    t_aw, ...
    z_aw, ...
    'LineWidth', 1.5);

yline( ...
    z_des, ...
    '--', ...
    'Desired Altitude');

grid on;

xlabel('Time [s]');
ylabel('Altitude z [m]');

title('PID Altitude Control: Anti-Windup Comparison');

legend( ...
    'Without Anti-Windup', ...
    'With Anti-Windup', ...
    'Desired Altitude');

%% ============================================================
%% Plot 2: Altitude error
%% ============================================================

figure;

plot( ...
    t_no_aw, ...
    error_no_aw, ...
    'LineWidth', 1.5);

hold on;

plot( ...
    t_aw, ...
    error_aw, ...
    'LineWidth', 1.5);

yline(0, '--');

grid on;

xlabel('Time [s]');
ylabel('Altitude Error [m]');

title('Altitude Error: Anti-Windup OFF vs ON');

legend( ...
    'Without Anti-Windup', ...
    'With Anti-Windup');

%% ============================================================
%% Plot 3: Integral state
%% ============================================================

figure;

plot( ...
    t_no_aw, ...
    x_no_aw(:,13), ...
    'LineWidth', 1.5);

hold on;

plot( ...
    t_aw, ...
    x_aw(:,13), ...
    'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Integral State [m*s]');

title('Integrator Windup Comparison');

legend( ...
    'Without Anti-Windup', ...
    'With Anti-Windup');

%% ============================================================
%% Plot 4: Motor thrust - no anti-windup
%% ============================================================

figure;

plot(t_no_aw, motor_no_aw(:,1), 'LineWidth', 1.3);
hold on;

plot(t_no_aw, motor_no_aw(:,2), 'LineWidth', 1.3);
plot(t_no_aw, motor_no_aw(:,3), 'LineWidth', 1.3);
plot(t_no_aw, motor_no_aw(:,4), 'LineWidth', 1.3);

yline( ...
    motor_thrust_max, ...
    '--', ...
    'Motor Limit');

grid on;

xlabel('Time [s]');
ylabel('Motor Thrust [N]');

title('Motor Thrust - Without Anti-Windup');

legend( ...
    'Motor 1', ...
    'Motor 2', ...
    'Motor 3', ...
    'Motor 4', ...
    'Maximum');

%% ============================================================
%% Plot 5: Motor thrust - with anti-windup
%% ============================================================

figure;

plot(t_aw, motor_aw(:,1), 'LineWidth', 1.3);
hold on;

plot(t_aw, motor_aw(:,2), 'LineWidth', 1.3);
plot(t_aw, motor_aw(:,3), 'LineWidth', 1.3);
plot(t_aw, motor_aw(:,4), 'LineWidth', 1.3);

yline( ...
    motor_thrust_max, ...
    '--', ...
    'Motor Limit');

grid on;

xlabel('Time [s]');
ylabel('Motor Thrust [N]');

title('Motor Thrust - With Anti-Windup');

legend( ...
    'Motor 1', ...
    'Motor 2', ...
    'Motor 3', ...
    'Motor 4', ...
    'Maximum');

%% ============================================================
%% Plot 6: Saturation status
%% ============================================================

figure;

stairs( ...
    t_no_aw, ...
    sat_no_aw, ...
    'LineWidth', 1.4);

hold on;

stairs( ...
    t_aw, ...
    sat_aw, ...
    'LineWidth', 1.4);

grid on;

ylim([-0.1 1.1]);

xlabel('Time [s]');
ylabel('Actuator Saturation');

yticks([0 1]);
yticklabels({'No','Yes'});

title('Actuator Saturation Comparison');

legend( ...
    'Without Anti-Windup', ...
    'With Anti-Windup');

%% ============================================================
%% PID WITHOUT ANTI-WINDUP
%% ============================================================

function [xdot_aug, motor_thrusts_sat, is_saturated] = ...
    altitude_pid_no_aw( ...
    t, x_aug, ...
    z_des, ...
    Kp_z, Ki_z, Kd_z, ...
    m, g, I, L, k_yaw, ...
    motor_thrust_min, motor_thrust_max)

x = x_aug(1:12);

integral_z = x_aug(13);

%% States

z = x(3);
vz = x(6);

%% Errors

e_z = z_des - z;
e_vz = -vz;

%% PID

T = ...
    m*g + ...
    Kp_z*e_z + ...
    Ki_z*integral_z + ...
    Kd_z*e_vz;

%% Keep vehicle level

tau_phi = 0;
tau_theta = 0;
tau_psi = 0;

%% Motor allocation

motor_thrusts_raw = motor_mixer( ...
    T, ...
    tau_phi, ...
    tau_theta, ...
    tau_psi, ...
    L, ...
    k_yaw);

%% Saturation

motor_thrusts_sat = max( ...
    min(motor_thrusts_raw, motor_thrust_max), ...
    motor_thrust_min);

is_saturated = any( ...
    abs(motor_thrusts_raw - motor_thrusts_sat) > 1e-9);

%% Actual applied control

u_actual = motor_unmixer( ...
    motor_thrusts_sat, ...
    L, ...
    k_yaw);

%% Dynamics

xdot = quad_dynamics( ...
    x, ...
    u_actual, ...
    m, ...
    g, ...
    I);

%% NO ANTI-WINDUP
% Integrator continues even during saturation

integral_z_dot = e_z;

%% Augmented dynamics

xdot_aug = [ ...
    xdot;
    integral_z_dot];

end

%% ============================================================
%% PID WITH ANTI-WINDUP
%% ============================================================

function [xdot_aug, motor_thrusts_sat, is_saturated] = ...
    altitude_pid_aw( ...
    t, x_aug, ...
    z_des, ...
    Kp_z, Ki_z, Kd_z, ...
    m, g, I, L, k_yaw, ...
    motor_thrust_min, motor_thrust_max)

x = x_aug(1:12);

integral_z = x_aug(13);

%% States

z = x(3);
vz = x(6);

%% Errors

e_z = z_des - z;
e_vz = -vz;

%% PID

T = ...
    m*g + ...
    Kp_z*e_z + ...
    Ki_z*integral_z + ...
    Kd_z*e_vz;

%% Keep vehicle level

tau_phi = 0;
tau_theta = 0;
tau_psi = 0;

%% Motor allocation

motor_thrusts_raw = motor_mixer( ...
    T, ...
    tau_phi, ...
    tau_theta, ...
    tau_psi, ...
    L, ...
    k_yaw);

%% Saturation

motor_thrusts_sat = max( ...
    min(motor_thrusts_raw, motor_thrust_max), ...
    motor_thrust_min);

is_saturated = any( ...
    abs(motor_thrusts_raw - motor_thrusts_sat) > 1e-9);

%% Actual applied control

u_actual = motor_unmixer( ...
    motor_thrusts_sat, ...
    L, ...
    k_yaw);

%% Dynamics

xdot = quad_dynamics( ...
    x, ...
    u_actual, ...
    m, ...
    g, ...
    I);

%% Conditional integration anti-windup

integral_z_dot = e_z;

if is_saturated

    T_requested = sum(motor_thrusts_raw);
    T_applied = sum(motor_thrusts_sat);

    % Upper motor saturation
    if T_requested > T_applied && e_z > 0

        integral_z_dot = 0;

    end

    % Lower motor saturation
    if T_requested < T_applied && e_z < 0

        integral_z_dot = 0;

    end

end

%% Augmented dynamics

xdot_aug = [ ...
    xdot;
    integral_z_dot];

end

%% ============================================================
%% Settling-time helper
%% ============================================================

function settling_time = calculate_settling_time( ...
    t, error_signal, threshold)

settling_time = NaN;

for k = 1:length(t)

    if all(abs(error_signal(k:end)) < threshold)

        settling_time = t(k);
        return;

    end

end

end