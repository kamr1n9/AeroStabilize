%% AeroStabilize - LQR Helix Trajectory Tracking
% LQR designed from hover linearization
% Applied to nonlinear 6-DOF quadcopter plant
% Uses moving reference states from helix_trajectory.m
% Includes motor allocation, saturation, tracking metrics and plots

clear;
clc;

addpath('../models');
addpath('../trajectories');

% Load parameters
run('../parameters/quadcopter_params.m');

%% ============================================================
%% Linearized hover model
%% ============================================================

A = zeros(12,12);

% Position derivatives
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

% Translational coupling around hover
A(4,8) = g;
A(5,7) = -g;

% Euler angle derivatives near hover
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

%% Input matrix

B = zeros(12,4);

B(6,1)  = 1/m;
B(10,2) = 1/I(1,1);
B(11,3) = 1/I(2,2);
B(12,4) = 1/I(3,3);

%% ============================================================
%% Controllability
%% ============================================================

Co = ctrb(A,B);
rank_Co = rank(Co);

fprintf('Controllability rank: %d / 12\n', rank_Co);

%% ============================================================
%% LQR tuning
%% ============================================================

Q = diag([ ...
    20, ...
    20, ...
    30, ...
    5, ...
    5, ...
    8, ...
    40, ...
    40, ...
    10, ...
    2, ...
    2, ...
    2]);

R = diag([ ...
    1.0, ...
    0.5, ...
    0.5, ...
    0.5]);

K = lqr(A,B,Q,R);

disp('LQR Gain Matrix K:');
disp(K);

%% ============================================================
%% Initial state
%% ============================================================

x0 = zeros(12,1);

% Start exactly at beginning of helix
[x_start, y_start, z_start, ...
 vx_start, vy_start, vz_start] = helix_trajectory(0);

x0(1) = x_start;
x0(2) = y_start;
x0(3) = z_start;

x0(4) = vx_start;
x0(5) = vy_start;
x0(6) = vz_start;

%% ============================================================
%% Simulation time
%% ============================================================

tspan = [0 20];

%% ============================================================
%% Closed-loop dynamics
%% ============================================================

dynamics = @(t,x) lqr_helix_closed_loop( ...
    t, x, K, ...
    m, g, I, L, k_yaw);

%% Run simulation

[t, x] = ode45(dynamics, tspan, x0);

%% ============================================================
%% Log reference, motor and control data
%% ============================================================

x_ref_log = zeros(length(t),12);

u_log = zeros(length(t),4);

motor_log = zeros(length(t),4);
motor_raw_log = zeros(length(t),4);

for k = 1:length(t)

    [~, ...
     x_ref_k, ...
     u_k, ...
     motor_k, ...
     motor_raw_k] = lqr_helix_closed_loop( ...
        t(k), ...
        x(k,:)', ...
        K, ...
        m, g, I, L, k_yaw);

    x_ref_log(k,:) = x_ref_k';

    u_log(k,:) = u_k';

    motor_log(k,:) = motor_k';
    motor_raw_log(k,:) = motor_raw_k';

end

%% ============================================================
%% Tracking errors
%% ============================================================

error_x = x_ref_log(:,1) - x(:,1);
error_y = x_ref_log(:,2) - x(:,2);
error_z = x_ref_log(:,3) - x(:,3);

position_error = sqrt( ...
    error_x.^2 + ...
    error_y.^2 + ...
    error_z.^2);

RMSE = sqrt(mean(position_error.^2));

max_error = max(position_error);

final_error = position_error(end);

%% ============================================================
%% Maximum attitude
%% ============================================================

max_roll = max(abs(rad2deg(x(:,7))));
max_pitch = max(abs(rad2deg(x(:,8))));
max_yaw = max(abs(rad2deg(x(:,9))));

%% ============================================================
%% Motor statistics
%% ============================================================

max_motor_thrust = max(motor_log, [], 'all');
min_motor_thrust = min(motor_log, [], 'all');

%% ============================================================
%% Motor saturation
%% ============================================================

motor_thrust_min = 0.0;
motor_thrust_max = 7.5;

saturation_log = any( ...
    motor_raw_log > motor_thrust_max | ...
    motor_raw_log < motor_thrust_min, ...
    2);

saturation_duration = trapz( ...
    t, ...
    double(saturation_log));

saturation_percentage = ...
    100 * saturation_duration / ...
    (t(end) - t(1));

%% ============================================================
%% Print results
%% ============================================================

fprintf('\n============================================\n');
fprintf('LQR HELIX TRACKING RESULTS\n');
fprintf('============================================\n');

fprintf('Trajectory RMSE: %.4f m\n', RMSE);
fprintf('Maximum Tracking Error: %.4f m\n', max_error);
fprintf('Final Tracking Error: %.4f m\n', final_error);

fprintf('\nMaximum Attitude\n');
fprintf('Roll:  %.2f deg\n', max_roll);
fprintf('Pitch: %.2f deg\n', max_pitch);
fprintf('Yaw:   %.2f deg\n', max_yaw);

fprintf('\nMotor Thrust\n');
fprintf('Maximum: %.3f N\n', max_motor_thrust);
fprintf('Minimum: %.3f N\n', min_motor_thrust);

fprintf('\nActuator Saturation\n');
fprintf('Saturation duration: %.3f s\n', ...
    saturation_duration);

fprintf('Time in motor saturation: %.2f %%\n', ...
    saturation_percentage);

%% ============================================================
%% Plot 1: 3D helix tracking
%% ============================================================

figure;

plot3( ...
    x_ref_log(:,1), ...
    x_ref_log(:,2), ...
    x_ref_log(:,3), ...
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

title('AeroStabilize - LQR Helix Trajectory Tracking');

legend( ...
    'Desired Trajectory', ...
    'Actual Trajectory');

view(3);

%% ============================================================
%% Plot 2: Position tracking error
%% ============================================================

figure;

plot( ...
    t, ...
    position_error, ...
    'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Position Error [m]');

title('LQR Helix Position Tracking Error');

%% ============================================================
%% Plot 3: x, y, z tracking
%% ============================================================

figure;

plot(t, x(:,1), 'LineWidth', 1.4);
hold on;

plot(t, x_ref_log(:,1), '--', 'LineWidth', 1.4);

plot(t, x(:,2), 'LineWidth', 1.4);
plot(t, x_ref_log(:,2), '--', 'LineWidth', 1.4);

plot(t, x(:,3), 'LineWidth', 1.4);
plot(t, x_ref_log(:,3), '--', 'LineWidth', 1.4);

grid on;

xlabel('Time [s]');
ylabel('Position [m]');

title('LQR Helix Position Tracking');

legend( ...
    'x Actual', ...
    'x Desired', ...
    'y Actual', ...
    'y Desired', ...
    'z Actual', ...
    'z Desired');

%% ============================================================
%% Plot 4: Attitude response
%% ============================================================

figure;

plot(t, rad2deg(x(:,7)), 'LineWidth', 1.4);
hold on;

plot(t, rad2deg(x(:,8)), 'LineWidth', 1.4);
plot(t, rad2deg(x(:,9)), 'LineWidth', 1.4);

grid on;

xlabel('Time [s]');
ylabel('Angle [deg]');

title('LQR Helix Attitude Response');

legend( ...
    'Roll', ...
    'Pitch', ...
    'Yaw');

%% ============================================================
%% Plot 5: Individual motor thrust
%% ============================================================

figure;

plot(t, motor_log(:,1), 'LineWidth', 1.3);
hold on;

plot(t, motor_log(:,2), 'LineWidth', 1.3);
plot(t, motor_log(:,3), 'LineWidth', 1.3);
plot(t, motor_log(:,4), 'LineWidth', 1.3);

yline( ...
    motor_thrust_max, ...
    '--', ...
    'Maximum Motor Thrust');

yline( ...
    motor_thrust_min, ...
    '--', ...
    'Minimum Motor Thrust');

grid on;

xlabel('Time [s]');
ylabel('Motor Thrust [N]');

title('LQR Helix - Individual Motor Thrust');

legend( ...
    'Motor 1', ...
    'Motor 2', ...
    'Motor 3', ...
    'Motor 4', ...
    'Upper Limit', ...
    'Lower Limit');

%% ============================================================
%% Plot 6: Saturation status
%% ============================================================

figure;

stairs( ...
    t, ...
    saturation_log, ...
    'LineWidth', 1.4);

grid on;

ylim([-0.1 1.1]);

xlabel('Time [s]');
ylabel('Motor Saturation');

yticks([0 1]);

yticklabels({ ...
    'No', ...
    'Yes'});

title('LQR Helix Motor Saturation Status');

%% ============================================================
%% LQR HELIX CLOSED-LOOP FUNCTION
%% ============================================================

function [ ...
    xdot, ...
    x_ref, ...
    u_actual, ...
    motor_thrusts_sat, ...
    motor_thrusts_raw] = ...
    lqr_helix_closed_loop( ...
    t, x, K, ...
    m, g, I, L, k_yaw)

%% ============================================================
%% Desired helix trajectory
%% ============================================================

[x_des, y_des, z_des, ...
 vx_des, vy_des, vz_des] = helix_trajectory(t);

%% Reference state

x_ref = zeros(12,1);

x_ref(1) = x_des;
x_ref(2) = y_des;
x_ref(3) = z_des;

x_ref(4) = vx_des;
x_ref(5) = vy_des;
x_ref(6) = vz_des;

% Desired roll/pitch/yaw around hover
x_ref(7) = 0;
x_ref(8) = 0;
x_ref(9) = 0;

% Desired body rates
x_ref(10) = 0;
x_ref(11) = 0;
x_ref(12) = 0;

%% ============================================================
%% LQR control
%% ============================================================

e = x - x_ref;

delta_u = -K * e;

delta_T   = delta_u(1);
tau_phi   = delta_u(2);
tau_theta = delta_u(3);
tau_psi   = delta_u(4);

%% Actual requested thrust

T = m*g + delta_T;

%% ============================================================
%% Motor allocation
%% ============================================================

motor_thrusts_raw = motor_mixer( ...
    T, ...
    tau_phi, ...
    tau_theta, ...
    tau_psi, ...
    L, ...
    k_yaw);

%% Motor saturation

motor_thrust_min = 0.0;
motor_thrust_max = 7.5;

motor_thrusts_sat = max( ...
    min( ...
        motor_thrusts_raw, ...
        motor_thrust_max), ...
    motor_thrust_min);

%% Actual applied control

u_actual = motor_unmixer( ...
    motor_thrusts_sat, ...
    L, ...
    k_yaw);

%% ============================================================
%% Nonlinear plant
%% ============================================================

xdot = quad_dynamics( ...
    x, ...
    u_actual, ...
    m, ...
    g, ...
    I);

end
