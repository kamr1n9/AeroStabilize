%% AeroStabilize - LQR Position Control
% Linearized quadcopter model around hover
% LQR applied to nonlinear 6-DOF plant
% Includes performance metrics and time-based actuator saturation analysis

clear;
clc;

addpath('../models');

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

% Translational coupling near hover
A(4,8) = g;
A(5,7) = -g;

% Euler angle derivatives
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
    20, ...   % x
    20, ...   % y
    30, ...   % z
    5,  ...   % vx
    5,  ...   % vy
    8,  ...   % vz
    40, ...   % roll
    40, ...   % pitch
    10, ...   % yaw
    2,  ...   % p
    2,  ...   % q
    2]);      % r

R = diag([ ...
    1.0, ...  % thrust deviation
    0.5, ...  % roll torque
    0.5, ...  % pitch torque
    0.5]);    % yaw torque

K = lqr(A,B,Q,R);

disp('LQR Gain Matrix K:');
disp(K);

%% ============================================================
%% Desired state
%% ============================================================

x_ref = zeros(12,1);

x_ref(1) = 1.0;
x_ref(2) = 1.0;
x_ref(3) = 2.0;

%% Initial state

x0 = zeros(12,1);

%% Simulation time

tspan = [0 10];

%% ============================================================
%% Closed-loop dynamics
%% ============================================================

dynamics = @(t,x) lqr_nonlinear_closed_loop( ...
    t, x, x_ref, K, ...
    m, g, I, L, k_yaw);

%% Run simulation

[t, x] = ode45(dynamics, tspan, x0);

%% ============================================================
%% Log controller and motor commands
%% ============================================================

u_log = zeros(length(t),4);

motor_log = zeros(length(t),4);
motor_raw_log = zeros(length(t),4);

for k = 1:length(t)

    [~, ...
     u_k, ...
     motor_k, ...
     motor_raw_k] = lqr_nonlinear_closed_loop( ...
        t(k), ...
        x(k,:)', ...
        x_ref, ...
        K, ...
        m, g, I, L, k_yaw);

    u_log(k,:) = u_k';

    motor_log(k,:) = motor_k';
    motor_raw_log(k,:) = motor_raw_k';

end

%% ============================================================
%% Tracking errors
%% ============================================================

error_x = x_ref(1) - x(:,1);
error_y = x_ref(2) - x(:,2);
error_z = x_ref(3) - x(:,3);

position_error = sqrt( ...
    error_x.^2 + ...
    error_y.^2 + ...
    error_z.^2);

RMSE = sqrt(mean(position_error.^2));

final_error = position_error(end);

%% ============================================================
%% Overshoot
%% ============================================================

overshoot_x = max( ...
    0, ...
    max(x(:,1)) - x_ref(1));

overshoot_y = max( ...
    0, ...
    max(x(:,2)) - x_ref(2));

overshoot_z = max( ...
    0, ...
    max(x(:,3)) - x_ref(3));

%% ============================================================
%% Settling time
%% ============================================================

settling_band = 0.02;

settling_x = calculate_settling_time( ...
    t, error_x, settling_band);

settling_y = calculate_settling_time( ...
    t, error_y, settling_band);

settling_z = calculate_settling_time( ...
    t, error_z, settling_band);

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
%% Motor saturation analysis
%% ============================================================

motor_thrust_min = 0.0;
motor_thrust_max = 7.5;

saturation_log = any( ...
    motor_raw_log > motor_thrust_max | ...
    motor_raw_log < motor_thrust_min, ...
    2);

%% Sample-based count

saturated_samples = sum(saturation_log);

%% Time-based saturation duration

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
fprintf('LQR POSITION CONTROL RESULTS\n');
fprintf('============================================\n');

fprintf('Position RMSE: %.4f m\n', RMSE);
fprintf('Final Position Error: %.4f m\n', final_error);

fprintf('\nOvershoot\n');
fprintf('x: %.4f m\n', overshoot_x);
fprintf('y: %.4f m\n', overshoot_y);
fprintf('z: %.4f m\n', overshoot_z);

fprintf('\nSettling Time (+/- %.2f m)\n', settling_band);
fprintf('x: %.3f s\n', settling_x);
fprintf('y: %.3f s\n', settling_y);
fprintf('z: %.3f s\n', settling_z);

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

fprintf('Saturated samples: %d / %d\n', ...
    saturated_samples, ...
    length(t));

%% ============================================================
%% Plot 1: Position response
%% ============================================================

figure;

plot(t, x(:,1), 'LineWidth', 1.5);
hold on;

plot(t, x(:,2), 'LineWidth', 1.5);
plot(t, x(:,3), 'LineWidth', 1.5);

yline(x_ref(1), '--');
yline(x_ref(2), '--');
yline(x_ref(3), '--');

grid on;

xlabel('Time [s]');
ylabel('Position [m]');

title('LQR Position Control');

legend( ...
    'x', ...
    'y', ...
    'z', ...
    'Desired x', ...
    'Desired y', ...
    'Desired z');

%% ============================================================
%% Plot 2: Position error
%% ============================================================

figure;

plot( ...
    t, ...
    position_error, ...
    'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Position Error [m]');

title('LQR Position Tracking Error');

%% ============================================================
%% Plot 3: Attitude response
%% ============================================================

figure;

plot( ...
    t, ...
    rad2deg(x(:,7)), ...
    'LineWidth', 1.5);

hold on;

plot( ...
    t, ...
    rad2deg(x(:,8)), ...
    'LineWidth', 1.5);

plot( ...
    t, ...
    rad2deg(x(:,9)), ...
    'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Angle [deg]');

title('LQR Attitude Response');

legend( ...
    'Roll', ...
    'Pitch', ...
    'Yaw');

%% ============================================================
%% Plot 4: Control inputs
%% ============================================================

figure;

plot(t, u_log(:,1), 'LineWidth', 1.5);
hold on;

plot(t, u_log(:,2), 'LineWidth', 1.5);
plot(t, u_log(:,3), 'LineWidth', 1.5);
plot(t, u_log(:,4), 'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Control Input');

title('LQR Control Inputs');

legend( ...
    'Total Thrust [N]', ...
    'Roll Torque [N*m]', ...
    'Pitch Torque [N*m]', ...
    'Yaw Torque [N*m]');

%% ============================================================
%% Plot 5: Individual motor thrust
%% ============================================================

figure;

plot(t, motor_log(:,1), 'LineWidth', 1.4);
hold on;

plot(t, motor_log(:,2), 'LineWidth', 1.4);
plot(t, motor_log(:,3), 'LineWidth', 1.4);
plot(t, motor_log(:,4), 'LineWidth', 1.4);

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

title('LQR Individual Motor Thrust');

legend( ...
    'Motor 1', ...
    'Motor 2', ...
    'Motor 3', ...
    'Motor 4', ...
    'Upper Limit', ...
    'Lower Limit');

%% ============================================================
%% Plot 6: Raw vs saturated motor commands
%% ============================================================

figure;

plot( ...
    t, ...
    motor_raw_log(:,1), ...
    '--', ...
    'LineWidth', 1.2);

hold on;

plot( ...
    t, ...
    motor_log(:,1), ...
    'LineWidth', 1.5);

yline( ...
    motor_thrust_max, ...
    '--', ...
    'Upper Limit');

yline( ...
    motor_thrust_min, ...
    '--', ...
    'Lower Limit');

grid on;

xlabel('Time [s]');
ylabel('Motor 1 Thrust [N]');

title('LQR Motor 1: Requested vs Applied');

legend( ...
    'Requested', ...
    'Applied', ...
    'Upper Limit', ...
    'Lower Limit');

%% ============================================================
%% Plot 7: Saturation status
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

title('LQR Motor Saturation Status');

%% ============================================================
%% NONLINEAR LQR CLOSED-LOOP FUNCTION
%% ============================================================

function [ ...
    xdot, ...
    u_actual, ...
    motor_thrusts_sat, ...
    motor_thrusts_raw] = ...
    lqr_nonlinear_closed_loop( ...
    t, x, x_ref, K, ...
    m, g, I, L, k_yaw)

%% State error

e = x - x_ref;

%% LQR control law

delta_u = -K * e;

delta_T   = delta_u(1);
tau_phi   = delta_u(2);
tau_theta = delta_u(3);
tau_psi   = delta_u(4);

%% Requested total thrust

T = m*g + delta_T;

%% Motor mixer

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

%% Actual control generated by motors

u_actual = motor_unmixer( ...
    motor_thrusts_sat, ...
    L, ...
    k_yaw);

%% Nonlinear plant

xdot = quad_dynamics( ...
    x, ...
    u_actual, ...
    m, ...
    g, ...
    I);

end

%% ============================================================
%% Settling-time helper
%% ============================================================

function settling_time = calculate_settling_time( ...
    t, ...
    error_signal, ...
    threshold)

settling_time = NaN;

for k = 1:length(t)

    if all( ...
        abs(error_signal(k:end)) < threshold)

        settling_time = t(k);
        return;

    end

end

end