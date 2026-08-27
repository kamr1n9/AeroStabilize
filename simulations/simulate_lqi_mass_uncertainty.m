%% AeroStabilize - LQI under +20% Mass Uncertainty
%
% LQI = LQR + Integral Action
%
% Goal:
% Eliminate steady-state altitude error caused by
% +20% mass model mismatch.
%
% Controller assumes:
%   m_nominal = 1.5 kg
%
% Actual nonlinear plant:
%   m_actual = 1.8 kg
%
% Augmented state:
%   [12 vehicle states ; integral_z]
%
% integral_z_dot = z_ref - z

clear;
clc;
close all;

addpath('../models');
addpath('../trajectories');

run('../parameters/quadcopter_params.m');

%% ============================================================
%% MASS UNCERTAINTY
%% ============================================================

m_nominal = m;

mass_increase = 0.20;

m_actual = ...
    m_nominal * ...
    (1 + mass_increase);

fprintf('Nominal mass: %.3f kg\n', m_nominal);
fprintf('Actual mass:  %.3f kg\n', m_actual);
fprintf('Mass error:   +%.1f %%\n', mass_increase*100);

%% ============================================================
%% COMMON SETTINGS
%% ============================================================

tspan = [0 25];

motor_thrust_min = 0.0;
motor_thrust_max = 7.5;

%% ============================================================
%% NOMINAL LINEAR MODEL
%% ============================================================

A = zeros(12,12);

A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

A(4,8) = g;
A(5,7) = -g;

A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

B = zeros(12,4);

B(6,1) = ...
    1/m_nominal;

B(10,2) = ...
    1/I(1,1);

B(11,3) = ...
    1/I(2,2);

B(12,4) = ...
    1/I(3,3);

%% ============================================================
%% AUGMENT MODEL WITH ALTITUDE INTEGRATOR
%% ============================================================

% integral state:
%
% xi_dot = z_ref - z
%
% For regulator design around zero error:
%
% xi_dot = -z_error
%
% Since x_error = x - x_ref:
%
% xi_dot = -x_error(3)

A_aug = zeros(13,13);

A_aug(1:12,1:12) = A;

% Integral state dynamics
A_aug(13,3) = -1;

B_aug = zeros(13,4);

B_aug(1:12,:) = B;

%% ============================================================
%% CONTROLLABILITY
%% ============================================================

Co_aug = ctrb( ...
    A_aug, ...
    B_aug);

rank_aug = rank(Co_aug);

fprintf( ...
    'Augmented controllability rank: %d / 13\n', ...
    rank_aug);

%% ============================================================
%% LQI WEIGHTING
%% ============================================================

% Same LQR state penalties as before
% plus a strong penalty on altitude integral state

Q_aug = diag([ ...
    20, ...   % x
    20, ...   % y
    30, ...   % z
    5,  ...   % vx
    5,  ...   % vy
    8,  ...   % vz
    40, ...   % phi
    40, ...   % theta
    10, ...   % psi
    2,  ...   % p
    2,  ...   % q
    2,  ...   % r
    25]);     % integral_z

R = diag([ ...
    1.0, ...
    0.5, ...
    0.5, ...
    0.5]);

%% ============================================================
%% LQI GAIN
%% ============================================================

K_aug = lqr( ...
    A_aug, ...
    B_aug, ...
    Q_aug, ...
    R);

%% Split gains

Kx = ...
    K_aug(:,1:12);

Ki = ...
    K_aug(:,13);

fprintf('\nLQI Integral Gain Vector:\n');
disp(Ki);

%% ============================================================
%% INITIAL STATE
%% ============================================================

% 12 vehicle states + integral state

x0 = zeros(13,1);

[x_start, ...
 y_start, ...
 z_start, ...
 vx_start, ...
 vy_start, ...
 vz_start] = ...
    helix_trajectory(0);

x0(1) = x_start;
x0(2) = y_start;
x0(3) = z_start;

x0(4) = vx_start;
x0(5) = vy_start;
x0(6) = vz_start;

% integral state starts at zero
x0(13) = 0;

%% ============================================================
%% RUN NONLINEAR LQI SIMULATION
%% ============================================================

dynamics = @(t,x) ...
    lqi_mass_closed_loop( ...
        t, ...
        x, ...
        Kx, ...
        Ki, ...
        m_nominal, ...
        m_actual, ...
        g, ...
        I, ...
        L, ...
        k_yaw, ...
        motor_thrust_min, ...
        motor_thrust_max);

[t,x] = ode45( ...
    dynamics, ...
    tspan, ...
    x0);

%% ============================================================
%% LOG DATA
%% ============================================================

N = length(t);

ref_log = zeros(N,12);

motor_log = zeros(N,4);
motor_raw_log = zeros(N,4);

control_log = zeros(N,4);

for k = 1:N

    [~, ...
     ref_k, ...
     motor_k, ...
     motor_raw_k, ...
     u_k] = ...
        lqi_mass_closed_loop( ...
            t(k), ...
            x(k,:)', ...
            Kx, ...
            Ki, ...
            m_nominal, ...
            m_actual, ...
            g, ...
            I, ...
            L, ...
            k_yaw, ...
            motor_thrust_min, ...
            motor_thrust_max);

    ref_log(k,:) = ...
        ref_k';

    motor_log(k,:) = ...
        motor_k';

    motor_raw_log(k,:) = ...
        motor_raw_k';

    control_log(k,:) = ...
        u_k';

end

%% ============================================================
%% TRACKING ERRORS
%% ============================================================

error_x = ...
    ref_log(:,1) - x(:,1);

error_y = ...
    ref_log(:,2) - x(:,2);

error_z = ...
    ref_log(:,3) - x(:,3);

position_error = sqrt( ...
    error_x.^2 + ...
    error_y.^2 + ...
    error_z.^2);

RMSE = ...
    sqrt(mean(position_error.^2));

max_error = ...
    max(position_error);

final_error = ...
    position_error(end);

final_altitude_error = ...
    error_z(end);

%% ============================================================
%% MOTOR STATISTICS
%% ============================================================

max_motor = ...
    max(motor_log,[], 'all');

min_motor = ...
    min(motor_log,[], 'all');

%% ============================================================
%% SATURATION ANALYSIS
%% ============================================================

saturation_log = any( ...
    motor_raw_log > motor_thrust_max | ...
    motor_raw_log < motor_thrust_min, ...
    2);

saturation_duration = ...
    trapz( ...
        t, ...
        double(saturation_log));

saturation_percentage = ...
    100 * saturation_duration / ...
    (t(end)-t(1));

%% ============================================================
%% PRINT RESULTS
%% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('LQI - +20%% MASS UNCERTAINTY RESULTS\n');
fprintf('============================================================\n');

fprintf('\nMass\n');

fprintf( ...
    'Nominal: %.3f kg\n', ...
    m_nominal);

fprintf( ...
    'Actual:  %.3f kg\n', ...
    m_actual);

fprintf('\nTracking\n');

fprintf( ...
    'RMSE:                 %.4f m\n', ...
    RMSE);

fprintf( ...
    'Maximum Error:        %.4f m\n', ...
    max_error);

fprintf( ...
    'Final Position Error: %.4f m\n', ...
    final_error);

fprintf( ...
    'Final Altitude Error: %.4f m\n', ...
    final_altitude_error);

fprintf('\nIntegral Action\n');

fprintf( ...
    'Final Integral State: %.4f m*s\n', ...
    x(end,13));

fprintf('\nMotor Thrust\n');

fprintf( ...
    'Maximum: %.3f N\n', ...
    max_motor);

fprintf( ...
    'Minimum: %.3f N\n', ...
    min_motor);

fprintf('\nSaturation\n');

fprintf( ...
    'Duration: %.3f s\n', ...
    saturation_duration);

fprintf( ...
    'Time: %.2f %%\n', ...
    saturation_percentage);

%% ============================================================
%% PLOT 1 - 3D TRAJECTORY
%% ============================================================

figure;

plot3( ...
    ref_log(:,1), ...
    ref_log(:,2), ...
    ref_log(:,3), ...
    '--', ...
    'LineWidth',1.5);

hold on;

plot3( ...
    x(:,1), ...
    x(:,2), ...
    x(:,3), ...
    'LineWidth',1.8);

grid on;

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

title( ...
    'LQI Helix Tracking with +20% Mass Uncertainty');

legend( ...
    'Desired', ...
    'LQI Actual');

view(3);

%% ============================================================
%% PLOT 2 - ALTITUDE
%% ============================================================

figure;

plot( ...
    t, ...
    ref_log(:,3), ...
    '--', ...
    'LineWidth',1.5);

hold on;

plot( ...
    t, ...
    x(:,3), ...
    'LineWidth',1.7);

grid on;

xlabel('Time [s]');
ylabel('Altitude z [m]');

title( ...
    'LQI Altitude Tracking under Mass Uncertainty');

legend( ...
    'Desired', ...
    'Actual');

%% ============================================================
%% PLOT 3 - ALTITUDE ERROR
%% ============================================================

figure;

plot( ...
    t, ...
    error_z, ...
    'LineWidth',1.6);

hold on;

yline( ...
    0, ...
    '--');

grid on;

xlabel('Time [s]');
ylabel('Altitude Error [m]');

title( ...
    'LQI Eliminates Steady-State Altitude Error');

%% ============================================================
%% PLOT 4 - INTEGRAL STATE
%% ============================================================

figure;

plot( ...
    t, ...
    x(:,13), ...
    'LineWidth',1.6);

grid on;

xlabel('Time [s]');
ylabel('Integral State [m*s]');

title( ...
    'LQI Altitude Integral State');

%% ============================================================
%% PLOT 5 - TOTAL THRUST
%% ============================================================

figure;

plot( ...
    t, ...
    control_log(:,1), ...
    'LineWidth',1.6);

hold on;

yline( ...
    m_nominal*g, ...
    '--', ...
    'Nominal Hover');

yline( ...
    m_actual*g, ...
    '--', ...
    'Actual Hover Required');

grid on;

xlabel('Time [s]');
ylabel('Total Thrust [N]');

title( ...
    'LQI Thrust Compensation');

legend( ...
    'Applied Total Thrust', ...
    'Nominal Hover', ...
    'Actual Hover Required');

%% ============================================================
%% PLOT 6 - POSITION ERROR
%% ============================================================

figure;

plot( ...
    t, ...
    position_error, ...
    'LineWidth',1.6);

grid on;

xlabel('Time [s]');
ylabel('3D Position Error [m]');

title( ...
    'LQI 3D Tracking Error');

%% ============================================================
%% PLOT 7 - MOTOR THRUST
%% ============================================================

figure;

plot(t,motor_log(:,1),'LineWidth',1.3);
hold on;

plot(t,motor_log(:,2),'LineWidth',1.3);
plot(t,motor_log(:,3),'LineWidth',1.3);
plot(t,motor_log(:,4),'LineWidth',1.3);

yline( ...
    motor_thrust_max, ...
    '--', ...
    'Motor Limit');

grid on;

xlabel('Time [s]');
ylabel('Motor Thrust [N]');

title( ...
    'LQI Individual Motor Thrust');

legend( ...
    'M1', ...
    'M2', ...
    'M3', ...
    'M4', ...
    'Upper Limit');

%% ============================================================
%% CLOSED LOOP FUNCTION
%% ============================================================

function [ ...
    xdot_aug, ...
    x_ref, ...
    motor_thrusts_sat, ...
    motor_thrusts_raw, ...
    u_actual] = ...
    lqi_mass_closed_loop( ...
    t, ...
    x_aug, ...
    Kx, ...
    Ki, ...
    m_nominal, ...
    m_actual, ...
    g, ...
    I, ...
    L, ...
    k_yaw, ...
    motor_thrust_min, ...
    motor_thrust_max)

%% ============================================================
%% VEHICLE + INTEGRAL STATES
%% ============================================================

x = ...
    x_aug(1:12);

xi = ...
    x_aug(13);

%% ============================================================
%% DESIRED TRAJECTORY
%% ============================================================

[x_des, ...
 y_des, ...
 z_des, ...
 vx_des, ...
 vy_des, ...
 vz_des, ...
 ax_ff, ...
 ay_ff, ...
 az_ff] = ...
    helix_trajectory(t);

%% ============================================================
%% FEEDFORWARD ATTITUDE
%% ============================================================

theta_ff = ...
    ax_ff / g;

phi_ff = ...
    -ay_ff / g;

max_ff_angle = ...
    deg2rad(15);

theta_ff = max( ...
    min(theta_ff,max_ff_angle), ...
    -max_ff_angle);

phi_ff = max( ...
    min(phi_ff,max_ff_angle), ...
    -max_ff_angle);

%% ============================================================
%% REFERENCE STATE
%% ============================================================

x_ref = zeros(12,1);

x_ref(1) = x_des;
x_ref(2) = y_des;
x_ref(3) = z_des;

x_ref(4) = vx_des;
x_ref(5) = vy_des;
x_ref(6) = vz_des;

x_ref(7) = phi_ff;
x_ref(8) = theta_ff;

%% ============================================================
%% STATE ERROR
%% ============================================================

e = ...
    x - x_ref;

%% ============================================================
%% LQI CONTROL LAW
%% ============================================================

delta_u = ...
    -Kx*e - Ki*xi;

delta_T = ...
    delta_u(1);

tau_phi = ...
    delta_u(2);

tau_theta = ...
    delta_u(3);

tau_psi = ...
    delta_u(4);

%% ============================================================
%% NOMINAL FEEDFORWARD
%% ============================================================

T_ff = ...
    m_nominal * ...
    (g + az_ff);

T = ...
    T_ff + delta_T;

%% ============================================================
%% MOTOR MIXER
%% ============================================================

motor_thrusts_raw = ...
    motor_mixer( ...
        T, ...
        tau_phi, ...
        tau_theta, ...
        tau_psi, ...
        L, ...
        k_yaw);

%% ============================================================
%% MOTOR LIMITS
%% ============================================================

motor_thrusts_sat = max( ...
    min( ...
        motor_thrusts_raw, ...
        motor_thrust_max), ...
    motor_thrust_min);

%% ============================================================
%% ACTUAL APPLIED CONTROL
%% ============================================================

u_actual = ...
    motor_unmixer( ...
        motor_thrusts_sat, ...
        L, ...
        k_yaw);

%% ============================================================
%% ACTUAL NONLINEAR PLANT
%% ============================================================

xdot = ...
    quad_dynamics( ...
        x, ...
        u_actual, ...
        m_actual, ...
        g, ...
        I);

%% ============================================================
%% INTEGRAL STATE
%% ============================================================

z_error = ...
    z_des - x(3);

xi_dot = ...
    z_error;

%% Augmented derivative

xdot_aug = [ ...
    xdot;
    xi_dot];

end