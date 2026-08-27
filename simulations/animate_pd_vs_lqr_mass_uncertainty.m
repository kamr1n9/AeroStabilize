%% AeroStabilize - PD vs LQR Mass Uncertainty Animation
% Split-screen 3D animation:
% Left  = PD + Feedforward
% Right = LQR + Feedforward
%
% Both controllers assume nominal mass = 1.5 kg
% Actual nonlinear plant uses +20% mass = 1.8 kg
%
% Animation shows:
% - Desired helix
% - Actual trajectory
% - 3D quadrotor orientation
% - Live 3D position error
% - Live altitude error
% - PD vs LQR side-by-side

clear;
clc;
close all;

addpath('../models');
addpath('../trajectories');

run('../parameters/quadcopter_params.m');

%% ============================================================
%% NOMINAL AND ACTUAL MASS
%% ============================================================

m_nominal = m;
mass_increase = 0.20;
m_actual = m_nominal * (1 + mass_increase);

fprintf('Nominal mass: %.3f kg\n', m_nominal);
fprintf('Actual mass:  %.3f kg\n', m_actual);
fprintf('Mass error:   +%.1f %%\n', mass_increase*100);

%% ============================================================
%% COMMON SETTINGS
%% ============================================================

tspan = [0 20];

motor_thrust_min = 0.0;
motor_thrust_max = 7.5;

%% ============================================================
%% INITIAL STATE
%% ============================================================

x0 = zeros(12,1);

[x_start, y_start, z_start, ...
 vx_start, vy_start, vz_start] = helix_trajectory(0);

x0(1) = x_start;
x0(2) = y_start;
x0(3) = z_start;

x0(4) = vx_start;
x0(5) = vy_start;
x0(6) = vz_start;

%% ============================================================
%% PD CONTROLLER GAINS
%% ============================================================

psi_des = 0;

Kp_x = 1.0;
Kd_x = 1.4;

Kp_y = 1.0;
Kd_y = 1.4;

Kp_z = 4.0;
Kd_z = 3.0;

Kp_phi = 0.8;
Kd_phi = 0.25;

Kp_theta = 0.8;
Kd_theta = 0.25;

Kp_psi = 0.5;
Kd_psi = 0.20;

%% ============================================================
%% LQR DESIGN USING NOMINAL MODEL
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

B(6,1)  = 1 / m_nominal;
B(10,2) = 1 / I(1,1);
B(11,3) = 1 / I(2,2);
B(12,4) = 1 / I(3,3);

Co = ctrb(A,B);

fprintf('LQR controllability rank: %d / 12\n', rank(Co));

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

%% ============================================================
%% RUN PD SIMULATION
%% ============================================================

pd_dynamics = @(t,x) ...
    pd_mass_uncertainty_closed_loop( ...
        t, x, ...
        psi_des, ...
        Kp_x, Kd_x, ...
        Kp_y, Kd_y, ...
        Kp_z, Kd_z, ...
        Kp_phi, Kd_phi, ...
        Kp_theta, Kd_theta, ...
        Kp_psi, Kd_psi, ...
        m_nominal, ...
        m_actual, ...
        g, I, L, k_yaw, ...
        motor_thrust_min, ...
        motor_thrust_max);

[t_pd, x_pd] = ode45(pd_dynamics, tspan, x0);

%% ============================================================
%% RUN LQR SIMULATION
%% ============================================================

lqr_dynamics = @(t,x) ...
    lqr_mass_uncertainty_closed_loop( ...
        t, x, ...
        K, ...
        m_nominal, ...
        m_actual, ...
        g, I, L, k_yaw, ...
        motor_thrust_min, ...
        motor_thrust_max);

[t_lqr, x_lqr] = ode45(lqr_dynamics, tspan, x0);

%% ============================================================
%% LOG PD DATA
%% ============================================================

N_pd = length(t_pd);

pd_ref = zeros(N_pd,12);

for k = 1:N_pd

    [~, ref_k] = ...
        pd_mass_uncertainty_closed_loop( ...
            t_pd(k), ...
            x_pd(k,:)', ...
            psi_des, ...
            Kp_x, Kd_x, ...
            Kp_y, Kd_y, ...
            Kp_z, Kd_z, ...
            Kp_phi, Kd_phi, ...
            Kp_theta, Kd_theta, ...
            Kp_psi, Kd_psi, ...
            m_nominal, ...
            m_actual, ...
            g, I, L, k_yaw, ...
            motor_thrust_min, ...
            motor_thrust_max);

    pd_ref(k,:) = ref_k';

end

%% ============================================================
%% LOG LQR DATA
%% ============================================================

N_lqr = length(t_lqr);

lqr_ref = zeros(N_lqr,12);

for k = 1:N_lqr

    [~, ref_k] = ...
        lqr_mass_uncertainty_closed_loop( ...
            t_lqr(k), ...
            x_lqr(k,:)', ...
            K, ...
            m_nominal, ...
            m_actual, ...
            g, I, L, k_yaw, ...
            motor_thrust_min, ...
            motor_thrust_max);

    lqr_ref(k,:) = ref_k';

end

%% ============================================================
%% ERRORS
%% ============================================================

pd_error_x = pd_ref(:,1) - x_pd(:,1);
pd_error_y = pd_ref(:,2) - x_pd(:,2);
pd_error_z = pd_ref(:,3) - x_pd(:,3);

pd_position_error = sqrt( ...
    pd_error_x.^2 + ...
    pd_error_y.^2 + ...
    pd_error_z.^2);

lqr_error_x = lqr_ref(:,1) - x_lqr(:,1);
lqr_error_y = lqr_ref(:,2) - x_lqr(:,2);
lqr_error_z = lqr_ref(:,3) - x_lqr(:,3);

lqr_position_error = sqrt( ...
    lqr_error_x.^2 + ...
    lqr_error_y.^2 + ...
    lqr_error_z.^2);

%% ============================================================
%% ANIMATION SETTINGS
%% ============================================================

frame_dt = 0.05;
animation_speed = 1.0;

t_anim = tspan(1):frame_dt:tspan(2);

x_pd_anim = interp1(t_pd, x_pd, t_anim, 'linear');
x_lqr_anim = interp1(t_lqr, x_lqr, t_anim, 'linear');

pd_ref_anim = interp1(t_pd, pd_ref, t_anim, 'linear');
lqr_ref_anim = interp1(t_lqr, lqr_ref, t_anim, 'linear');

pd_error_anim = interp1(t_pd, pd_position_error, t_anim, 'linear');
lqr_error_anim = interp1(t_lqr, lqr_position_error, t_anim, 'linear');

pd_alt_error_anim = interp1(t_pd, pd_error_z, t_anim, 'linear');
lqr_alt_error_anim = interp1(t_lqr, lqr_error_z, t_anim, 'linear');

%% ============================================================
%% QUADCOPTER VISUAL GEOMETRY
%% ============================================================

arm_visual = 0.35;
rotor_radius = 0.10;
rotor_points = 30;

motor1_body = [ arm_visual; 0; 0];
motor2_body = [0;  arm_visual; 0];
motor3_body = [-arm_visual; 0; 0];
motor4_body = [0; -arm_visual; 0];

rotor_angle = linspace(0, 2*pi, rotor_points);

rotor_circle = [ ...
    rotor_radius*cos(rotor_angle);
    rotor_radius*sin(rotor_angle);
    zeros(1,rotor_points)];

%% ============================================================
%% FIGURE
%% ============================================================

fig = figure('Name', 'AeroStabilize - PD vs LQR Mass Uncertainty');
fig.Position = [80 60 1450 800];

%% ============================================================
%% LEFT AXIS - PD
%% ============================================================

ax_pd = axes('Parent', fig, 'Position', [0.05 0.33 0.40 0.60]);
hold(ax_pd, 'on');
grid(ax_pd, 'on');
axis(ax_pd, 'equal');

xlabel(ax_pd, 'x [m]');
ylabel(ax_pd, 'y [m]');
zlabel(ax_pd, 'z [m]');

title(ax_pd, 'PD + Feedforward');
view(ax_pd, 45, 25);

%% ============================================================
%% RIGHT AXIS - LQR
%% ============================================================

ax_lqr = axes('Parent', fig, 'Position', [0.55 0.33 0.40 0.60]);
hold(ax_lqr, 'on');
grid(ax_lqr, 'on');
axis(ax_lqr, 'equal');

xlabel(ax_lqr, 'x [m]');
ylabel(ax_lqr, 'y [m]');
zlabel(ax_lqr, 'z [m]');

title(ax_lqr, 'LQR + Feedforward');
view(ax_lqr, 45, 25);

%% ============================================================
%% DESIRED TRAJECTORY ON BOTH AXES
%% ============================================================

plot3(ax_pd, ...
    pd_ref_anim(:,1), pd_ref_anim(:,2), pd_ref_anim(:,3), ...
    '--', 'LineWidth', 1.4);

plot3(ax_lqr, ...
    lqr_ref_anim(:,1), lqr_ref_anim(:,2), lqr_ref_anim(:,3), ...
    '--', 'LineWidth', 1.4);

plot3(ax_pd, ...
    x_pd_anim(:,1), x_pd_anim(:,2), x_pd_anim(:,3), ...
    ':', 'LineWidth', 1.0);

plot3(ax_lqr, ...
    x_lqr_anim(:,1), x_lqr_anim(:,2), x_lqr_anim(:,3), ...
    ':', 'LineWidth', 1.0);

%% Common axis limits

all_ref_x = [pd_ref_anim(:,1); lqr_ref_anim(:,1)];
all_ref_y = [pd_ref_anim(:,2); lqr_ref_anim(:,2)];
all_ref_z = [pd_ref_anim(:,3); lqr_ref_anim(:,3)];

margin = 1.0;

xmin = min(all_ref_x) - margin;
xmax = max(all_ref_x) + margin;

ymin = min(all_ref_y) - margin;
ymax = max(all_ref_y) + margin;

zmin = min(all_ref_z) - 1.0;
zmax = max(all_ref_z) + 0.8;

xlim(ax_pd, [xmin xmax]);
ylim(ax_pd, [ymin ymax]);
zlim(ax_pd, [zmin zmax]);

xlim(ax_lqr, [xmin xmax]);
ylim(ax_lqr, [ymin ymax]);
zlim(ax_lqr, [zmin zmax]);

%% ============================================================
%% GRAPHICS OBJECTS - PD
%% ============================================================

pd_path = plot3(ax_pd, NaN, NaN, NaN, 'LineWidth', 2.0);

pd_arm1 = plot3(ax_pd, NaN, NaN, NaN, 'LineWidth', 3);
pd_arm2 = plot3(ax_pd, NaN, NaN, NaN, 'LineWidth', 3);

pd_body = plot3(ax_pd, NaN, NaN, NaN, ...
    'o', 'MarkerSize', 9, 'MarkerFaceColor', 'auto');

pd_rotor1 = plot3(ax_pd, NaN, NaN, NaN, 'LineWidth', 1.5);
pd_rotor2 = plot3(ax_pd, NaN, NaN, NaN, 'LineWidth', 1.5);
pd_rotor3 = plot3(ax_pd, NaN, NaN, NaN, 'LineWidth', 1.5);
pd_rotor4 = plot3(ax_pd, NaN, NaN, NaN, 'LineWidth', 1.5);

pd_desired_point = plot3(ax_pd, NaN, NaN, NaN, ...
    'x', 'MarkerSize', 11, 'LineWidth', 2);

pd_status = text(ax_pd, 0.02, 0.97, '', ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'top', ...
    'FontSize', 10, ...
    'FontWeight', 'bold');

%% ============================================================
%% GRAPHICS OBJECTS - LQR
%% ============================================================

lqr_path = plot3(ax_lqr, NaN, NaN, NaN, 'LineWidth', 2.0);

lqr_arm1 = plot3(ax_lqr, NaN, NaN, NaN, 'LineWidth', 3);
lqr_arm2 = plot3(ax_lqr, NaN, NaN, NaN, 'LineWidth', 3);

lqr_body = plot3(ax_lqr, NaN, NaN, NaN, ...
    'o', 'MarkerSize', 9, 'MarkerFaceColor', 'auto');

lqr_rotor1 = plot3(ax_lqr, NaN, NaN, NaN, 'LineWidth', 1.5);
lqr_rotor2 = plot3(ax_lqr, NaN, NaN, NaN, 'LineWidth', 1.5);
lqr_rotor3 = plot3(ax_lqr, NaN, NaN, NaN, 'LineWidth', 1.5);
lqr_rotor4 = plot3(ax_lqr, NaN, NaN, NaN, 'LineWidth', 1.5);

lqr_desired_point = plot3(ax_lqr, NaN, NaN, NaN, ...
    'x', 'MarkerSize', 11, 'LineWidth', 2);

lqr_status = text(ax_lqr, 0.02, 0.97, '', ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'top', ...
    'FontSize', 10, ...
    'FontWeight', 'bold');

%% ============================================================
%% BOTTOM LEFT - 3D POSITION ERROR
%% ============================================================

ax_err = axes('Parent', fig, 'Position', [0.07 0.08 0.38 0.17]);
hold(ax_err, 'on');
grid(ax_err, 'on');

plot(ax_err, t_anim, pd_error_anim, 'LineWidth', 1.3);
plot(ax_err, t_anim, lqr_error_anim, 'LineWidth', 1.3);

pd_err_marker = plot(ax_err, NaN, NaN, ...
    'o', 'MarkerSize', 7, 'MarkerFaceColor', 'auto');

lqr_err_marker = plot(ax_err, NaN, NaN, ...
    'o', 'MarkerSize', 7, 'MarkerFaceColor', 'auto');

xlabel(ax_err, 'Time [s]');
ylabel(ax_err, '3D Position Error [m]');
title(ax_err, 'Live 3D Position Error');

legend(ax_err, 'PD', 'LQR', 'PD current', 'LQR current', ...
    'Location', 'best');

xlim(ax_err, [tspan(1) tspan(2)]);

%% ============================================================
%% BOTTOM RIGHT - ALTITUDE ERROR
%% ============================================================

ax_alt = axes('Parent', fig, 'Position', [0.57 0.08 0.38 0.17]);
hold(ax_alt, 'on');
grid(ax_alt, 'on');

plot(ax_alt, t_anim, pd_alt_error_anim, 'LineWidth', 1.3);
plot(ax_alt, t_anim, lqr_alt_error_anim, 'LineWidth', 1.3);

yline(ax_alt, 0, '--');

pd_alt_marker = plot(ax_alt, NaN, NaN, ...
    'o', 'MarkerSize', 7, 'MarkerFaceColor', 'auto');

lqr_alt_marker = plot(ax_alt, NaN, NaN, ...
    'o', 'MarkerSize', 7, 'MarkerFaceColor', 'auto');

xlabel(ax_alt, 'Time [s]');
ylabel(ax_alt, 'Altitude Error [m]');
title(ax_alt, 'Live Altitude Error');

legend(ax_alt, 'PD', 'LQR', 'Zero', ...
    'PD current', 'LQR current', ...
    'Location', 'best');

xlim(ax_alt, [tspan(1) tspan(2)]);

%% ============================================================
%% ANIMATION LOOP
%% ============================================================

for k = 1:length(t_anim)

    current_time = t_anim(k);

    %% PD state
    pd_position = [x_pd_anim(k,1); x_pd_anim(k,2); x_pd_anim(k,3)];
    pd_phi   = x_pd_anim(k,7);
    pd_theta = x_pd_anim(k,8);
    pd_psi   = x_pd_anim(k,9);

    %% LQR state
    lqr_position = [x_lqr_anim(k,1); x_lqr_anim(k,2); x_lqr_anim(k,3)];
    lqr_phi   = x_lqr_anim(k,7);
    lqr_theta = x_lqr_anim(k,8);
    lqr_psi   = x_lqr_anim(k,9);

    %% Rotation matrices PD
    R_pd = euler_to_rot(pd_phi, pd_theta, pd_psi);

    %% Rotation matrices LQR
    R_lqr = euler_to_rot(lqr_phi, lqr_theta, lqr_psi);

    %% Update PD drone
    update_drone( ...
        pd_position, R_pd, ...
        motor1_body, motor2_body, motor3_body, motor4_body, ...
        rotor_circle, ...
        pd_arm1, pd_arm2, pd_body, ...
        pd_rotor1, pd_rotor2, pd_rotor3, pd_rotor4);

    %% Update LQR drone
    update_drone( ...
        lqr_position, R_lqr, ...
        motor1_body, motor2_body, motor3_body, motor4_body, ...
        rotor_circle, ...
        lqr_arm1, lqr_arm2, lqr_body, ...
        lqr_rotor1, lqr_rotor2, lqr_rotor3, lqr_rotor4);

    %% Update travelled paths
    set(pd_path, ...
        'XData', x_pd_anim(1:k,1), ...
        'YData', x_pd_anim(1:k,2), ...
        'ZData', x_pd_anim(1:k,3));

    set(lqr_path, ...
        'XData', x_lqr_anim(1:k,1), ...
        'YData', x_lqr_anim(1:k,2), ...
        'ZData', x_lqr_anim(1:k,3));

    %% Update desired points
    set(pd_desired_point, ...
        'XData', pd_ref_anim(k,1), ...
        'YData', pd_ref_anim(k,2), ...
        'ZData', pd_ref_anim(k,3));

    set(lqr_desired_point, ...
        'XData', lqr_ref_anim(k,1), ...
        'YData', lqr_ref_anim(k,2), ...
        'ZData', lqr_ref_anim(k,3));

    %% Status texts
    pd_status_string = sprintf([ ...
        'Controller: PD + Feedforward\n' ...
        'Time: %.2f s\n' ...
        'Model mass: %.2f kg\n' ...
        'Actual mass: %.2f kg\n' ...
        '3D error: %.3f m\n' ...
        'Alt. error: %.3f m\n' ...
        'Roll: %.1f deg\n' ...
        'Pitch: %.1f deg'], ...
        current_time, ...
        m_nominal, ...
        m_actual, ...
        pd_error_anim(k), ...
        pd_alt_error_anim(k), ...
        rad2deg(pd_phi), ...
        rad2deg(pd_theta));

    lqr_status_string = sprintf([ ...
        'Controller: LQR + Feedforward\n' ...
        'Time: %.2f s\n' ...
        'Model mass: %.2f kg\n' ...
        'Actual mass: %.2f kg\n' ...
        '3D error: %.3f m\n' ...
        'Alt. error: %.3f m\n' ...
        'Roll: %.1f deg\n' ...
        'Pitch: %.1f deg'], ...
        current_time, ...
        m_nominal, ...
        m_actual, ...
        lqr_error_anim(k), ...
        lqr_alt_error_anim(k), ...
        rad2deg(lqr_phi), ...
        rad2deg(lqr_theta));

    set(pd_status, 'String', pd_status_string);
    set(lqr_status, 'String', lqr_status_string);

    %% Update error markers
    set(pd_err_marker, 'XData', current_time, 'YData', pd_error_anim(k));
    set(lqr_err_marker, 'XData', current_time, 'YData', lqr_error_anim(k));

    set(pd_alt_marker, 'XData', current_time, 'YData', pd_alt_error_anim(k));
    set(lqr_alt_marker, 'XData', current_time, 'YData', lqr_alt_error_anim(k));

    %% Global title
    sgtitle(fig, ...
        sprintf('AeroStabilize | +20%% Mass Uncertainty | t = %.2f s', current_time), ...
        'FontWeight', 'bold');

    drawnow;
    pause(frame_dt / animation_speed);

end

%% ============================================================
%% PD CLOSED LOOP
%% ============================================================

function [xdot, x_ref, motor_thrusts_sat, motor_thrusts_raw] = ...
    pd_mass_uncertainty_closed_loop( ...
    t, x, ...
    psi_des, ...
    Kp_x, Kd_x, ...
    Kp_y, Kd_y, ...
    Kp_z, Kd_z, ...
    Kp_phi, Kd_phi, ...
    Kp_theta, Kd_theta, ...
    Kp_psi, Kd_psi, ...
    m_nominal, ...
    m_actual, ...
    g, I, L, k_yaw, ...
    motor_thrust_min, ...
    motor_thrust_max)

[x_des, y_des, z_des, ...
 vx_des, vy_des, vz_des, ...
 ax_ff, ay_ff, az_ff] = helix_trajectory(t);

x_ref = zeros(12,1);

x_ref(1) = x_des;
x_ref(2) = y_des;
x_ref(3) = z_des;

x_ref(4) = vx_des;
x_ref(5) = vy_des;
x_ref(6) = vz_des;

x_pos = x(1);
y_pos = x(2);
z_pos = x(3);

vx = x(4);
vy = x(5);
vz = x(6);

phi = x(7);
theta = x(8);
psi = x(9);

p = x(10);
q = x(11);
r = x(12);

e_x = x_des - x_pos;
e_y = y_des - y_pos;
e_z = z_des - z_pos;

e_vx = vx_des - vx;
e_vy = vy_des - vy;
e_vz = vz_des - vz;

ax_des = ax_ff + Kp_x*e_x + Kd_x*e_vx;
ay_des = ay_ff + Kp_y*e_y + Kd_y*e_vy;

theta_des = ax_des / g;
phi_des = -ay_des / g;

max_angle = deg2rad(20);

theta_des = max(min(theta_des, max_angle), -max_angle);
phi_des   = max(min(phi_des, max_angle), -max_angle);

T = m_nominal*g + m_nominal*az_ff + Kp_z*e_z + Kd_z*e_vz;

tau_phi = Kp_phi*(phi_des - phi)   - Kd_phi*p;
tau_theta = Kp_theta*(theta_des - theta) - Kd_theta*q;
tau_psi = Kp_psi*(psi_des - psi)   - Kd_psi*r;

motor_thrusts_raw = motor_mixer( ...
    T, tau_phi, tau_theta, tau_psi, L, k_yaw);

motor_thrusts_sat = max( ...
    min(motor_thrusts_raw, motor_thrust_max), ...
    motor_thrust_min);

u_actual = motor_unmixer(motor_thrusts_sat, L, k_yaw);

xdot = quad_dynamics(x, u_actual, m_actual, g, I);

end

%% ============================================================
%% LQR CLOSED LOOP
%% ============================================================

function [xdot, x_ref, motor_thrusts_sat, motor_thrusts_raw] = ...
    lqr_mass_uncertainty_closed_loop( ...
    t, x, ...
    K, ...
    m_nominal, ...
    m_actual, ...
    g, I, L, k_yaw, ...
    motor_thrust_min, ...
    motor_thrust_max)

[x_des, y_des, z_des, ...
 vx_des, vy_des, vz_des, ...
 ax_ff, ay_ff, az_ff] = helix_trajectory(t);

theta_ff = ax_ff / g;
phi_ff   = -ay_ff / g;

max_ff_angle = deg2rad(15);

theta_ff = max(min(theta_ff, max_ff_angle), -max_ff_angle);
phi_ff   = max(min(phi_ff, max_ff_angle), -max_ff_angle);

x_ref = zeros(12,1);

x_ref(1) = x_des;
x_ref(2) = y_des;
x_ref(3) = z_des;

x_ref(4) = vx_des;
x_ref(5) = vy_des;
x_ref(6) = vz_des;

x_ref(7) = phi_ff;
x_ref(8) = theta_ff;
x_ref(9) = 0;

x_ref(10) = 0;
x_ref(11) = 0;
x_ref(12) = 0;

e = x - x_ref;

delta_u = -K * e;

delta_T   = delta_u(1);
tau_phi   = delta_u(2);
tau_theta = delta_u(3);
tau_psi   = delta_u(4);

T_ff = m_nominal * (g + az_ff);
T = T_ff + delta_T;

motor_thrusts_raw = motor_mixer( ...
    T, tau_phi, tau_theta, tau_psi, L, k_yaw);

motor_thrusts_sat = max( ...
    min(motor_thrusts_raw, motor_thrust_max), ...
    motor_thrust_min);

u_actual = motor_unmixer(motor_thrusts_sat, L, k_yaw);

xdot = quad_dynamics(x, u_actual, m_actual, g, I);

end

%% ============================================================
%% EULER TO ROTATION MATRIX
%% ============================================================

function R = euler_to_rot(phi, theta, psi)

Rx = [ ...
    1, 0, 0;
    0, cos(phi), -sin(phi);
    0, sin(phi), cos(phi)];

Ry = [ ...
    cos(theta), 0, sin(theta);
    0, 1, 0;
    -sin(theta), 0, cos(theta)];

Rz = [ ...
    cos(psi), -sin(psi), 0;
    sin(psi),  cos(psi), 0;
    0, 0, 1];

R = Rz * Ry * Rx;

end

%% ============================================================
%% UPDATE DRONE DRAWING
%% ============================================================

function update_drone( ...
    position, Rbw, ...
    motor1_body, motor2_body, motor3_body, motor4_body, ...
    rotor_circle, ...
    arm1_plot, arm2_plot, body_plot, ...
    rotor1_plot, rotor2_plot, rotor3_plot, rotor4_plot)

motor1_world = position + Rbw * motor1_body;
motor2_world = position + Rbw * motor2_body;
motor3_world = position + Rbw * motor3_body;
motor4_world = position + Rbw * motor4_body;

set(arm1_plot, ...
    'XData', [motor1_world(1), motor3_world(1)], ...
    'YData', [motor1_world(2), motor3_world(2)], ...
    'ZData', [motor1_world(3), motor3_world(3)]);

set(arm2_plot, ...
    'XData', [motor2_world(1), motor4_world(1)], ...
    'YData', [motor2_world(2), motor4_world(2)], ...
    'ZData', [motor2_world(3), motor4_world(3)]);

set(body_plot, ...
    'XData', position(1), ...
    'YData', position(2), ...
    'ZData', position(3));

rotor_world = Rbw * rotor_circle;

rotor1 = rotor_world + motor1_world;
rotor2 = rotor_world + motor2_world;
rotor3 = rotor_world + motor3_world;
rotor4 = rotor_world + motor4_world;

set(rotor1_plot, ...
    'XData', rotor1(1,:), ...
    'YData', rotor1(2,:), ...
    'ZData', rotor1(3,:));

set(rotor2_plot, ...
    'XData', rotor2(1,:), ...
    'YData', rotor2(2,:), ...
    'ZData', rotor2(3,:));

set(rotor3_plot, ...
    'XData', rotor3(1,:), ...
    'YData', rotor3(2,:), ...
    'ZData', rotor3(3,:));

set(rotor4_plot, ...
    'XData', rotor4(1,:), ...
    'YData', rotor4(2,:), ...
    'ZData', rotor4(3,:));

end