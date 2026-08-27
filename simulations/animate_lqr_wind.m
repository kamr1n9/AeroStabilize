%% AeroStabilize
% LQR + Feedforward Helix Tracking
% 3D Animation with 5 N Wind Disturbance
%
% Wind disturbance:
%   +5 N in world X direction
%   active from t = 8 s to t = 9 s
%
% Animation displays:
%   - Desired helix
%   - Actual trajectory
%   - 3D quadcopter attitude
%   - Wind-force arrow
%   - Tracking error
%   - Roll / pitch
%   - Individual motor thrusts

clear;
clc;
close all;

%% ============================================================
% PARAMETERS
% ============================================================

g = 9.81;

m = 1.5;

L = 0.225;

Ix = 0.020;
Iy = 0.020;
Iz = 0.040;

I = diag([Ix Iy Iz]);

k_yaw = 0.02;

%% Motor limits

T_motor_min = 0.0;
T_motor_max = 7.5;

%% ============================================================
% HELIX TRAJECTORY PARAMETERS
% ============================================================

R_helix = 2.0;
omega_helix = 0.4;

z0_helix = 1.0;
vz_helix = 0.15;

%% ============================================================
% WIND DISTURBANCE
% ============================================================

wind_start = 8.0;
wind_end   = 9.0;

F_wind = [ ...
    5.0;
    0.0;
    0.0];

%% ============================================================
% LINEARIZED HOVER MODEL
% ============================================================

A = zeros(12);

% Position kinematics
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

% Horizontal acceleration coupling
A(4,8) = g;
A(5,7) = -g;

% Euler angle kinematics
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

B = zeros(12,4);

% Thrust
B(6,1) = 1/m;

% Torques
B(10,2) = 1/Ix;
B(11,3) = 1/Iy;
B(12,4) = 1/Iz;

%% ============================================================
% CONTROLLABILITY
% ============================================================

Co = ctrb(A,B);

rank_Co = rank(Co);

fprintf('============================================\n');
fprintf('LQR + FF Wind Animation\n');
fprintf('============================================\n');

fprintf('Controllability rank: %d / 12\n',rank_Co);

%% ============================================================
% LQR WEIGHTS
% ============================================================

Q = diag([ ...
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
    2]);      % r

R_lqr = diag([ ...
    1.0, ...
    0.5, ...
    0.5, ...
    0.5]);

K = lqr(A,B,Q,R_lqr);

%% ============================================================
% SIMULATION SETTINGS
% ============================================================

t_start = 0;
t_end   = 20;

%% Initial trajectory state

[xd0,yd0,zd0, ...
 vxd0,vyd0,vzd0, ...
 ax0,ay0,~] = helix_reference( ...
    0, ...
    R_helix, ...
    omega_helix, ...
    z0_helix, ...
    vz_helix);

phi_ff_0   = -ay0/g;
theta_ff_0 =  ax0/g;

x_initial = zeros(12,1);

x_initial(1) = xd0;
x_initial(2) = yd0;
x_initial(3) = zd0;

x_initial(4) = vxd0;
x_initial(5) = vyd0;
x_initial(6) = vzd0;

x_initial(7) = phi_ff_0;
x_initial(8) = theta_ff_0;
x_initial(9) = 0;

%% ============================================================
% RUN NONLINEAR SIMULATION
% ============================================================

odefun = @(t,x) quad_lqr_wind_dynamics( ...
    t, ...
    x, ...
    K, ...
    m, ...
    g, ...
    I, ...
    L, ...
    k_yaw, ...
    T_motor_min, ...
    T_motor_max, ...
    R_helix, ...
    omega_helix, ...
    z0_helix, ...
    vz_helix, ...
    wind_start, ...
    wind_end, ...
    F_wind);

options = odeset( ...
    'RelTol',1e-7, ...
    'AbsTol',1e-9);

[t,X] = ode45( ...
    odefun, ...
    [t_start t_end], ...
    x_initial, ...
    options);

%% ============================================================
% EXTRACT STATES
% ============================================================

x = X(:,1);
y = X(:,2);
z = X(:,3);

phi   = X(:,7);
theta = X(:,8);
psi   = X(:,9);

%% ============================================================
% RECONSTRUCT REFERENCE + CONTROL HISTORY
% ============================================================

N = length(t);

x_des_log = zeros(N,1);
y_des_log = zeros(N,1);
z_des_log = zeros(N,1);

tracking_error = zeros(N,1);

motor_log = zeros(N,4);

T_total_log = zeros(N,1);

wind_log = zeros(N,3);

saturation_log = false(N,1);

for k = 1:N

    tk = t(k);

    state = X(k,:)';

    %% Reference trajectory

    [xd,yd,zd, ...
     vxd,vyd,vzd, ...
     ax_ff,ay_ff,az_ff] = helix_reference( ...
        tk, ...
        R_helix, ...
        omega_helix, ...
        z0_helix, ...
        vz_helix);

    x_des_log(k) = xd;
    y_des_log(k) = yd;
    z_des_log(k) = zd;

    %% Acceleration feedforward attitude

    theta_ff = ax_ff/g;
    phi_ff   = -ay_ff/g;

    %% Moving reference state

    x_ref = zeros(12,1);

    x_ref(1) = xd;
    x_ref(2) = yd;
    x_ref(3) = zd;

    x_ref(4) = vxd;
    x_ref(5) = vyd;
    x_ref(6) = vzd;

    x_ref(7) = phi_ff;
    x_ref(8) = theta_ff;
    x_ref(9) = 0;

    %% LQR feedback

    e = state - x_ref;

    delta_u = -K*e;

    %% Feedforward control

    T_ff = m*(g + az_ff);

    u_cmd = [ ...
        T_ff;
        0;
        0;
        0] + delta_u;

    %% Mixer

    M = [ ...
        1,      1,      1,      1;
        0,      L,      0,     -L;
       -L,      0,      L,      0;
        k_yaw, -k_yaw,  k_yaw, -k_yaw];

    motor_requested = M\u_cmd;

    %% Saturation detection

    saturation_log(k) = any( ...
        motor_requested < T_motor_min | ...
        motor_requested > T_motor_max);

    %% Apply motor limits

    motor_thrust = min( ...
        max(motor_requested,T_motor_min), ...
        T_motor_max);

    motor_log(k,:) = motor_thrust';

    %% Achievable control

    u_actual = M*motor_thrust;

    T_total_log(k) = u_actual(1);

    %% Tracking error

    tracking_error(k) = norm( ...
        state(1:3) - ...
        [xd;yd;zd]);

    %% Wind

    if tk >= wind_start && tk <= wind_end
        wind_log(k,:) = F_wind';
    end

end

%% ============================================================
% PERFORMANCE RESULTS
% ============================================================

RMSE = sqrt(mean(tracking_error.^2));

max_error = max(tracking_error);

final_error = tracking_error(end);

max_motor = max(motor_log,[],'all');
min_motor = min(motor_log,[],'all');

saturation_duration = trapz( ...
    t, ...
    double(saturation_log));

saturation_percentage = ...
    100*saturation_duration/(t(end)-t(1));

fprintf('\n');
fprintf('============================================\n');
fprintf('Simulation Results\n');
fprintf('============================================\n');

fprintf('RMSE: %.4f m\n',RMSE);
fprintf('Maximum tracking error: %.4f m\n',max_error);
fprintf('Final tracking error: %.4f m\n',final_error);

fprintf('\n');

fprintf('Maximum roll: %.2f deg\n', ...
    max(abs(rad2deg(phi))));

fprintf('Maximum pitch: %.2f deg\n', ...
    max(abs(rad2deg(theta))));

fprintf('\n');

fprintf('Maximum motor thrust: %.3f N\n',max_motor);
fprintf('Minimum motor thrust: %.3f N\n',min_motor);

fprintf('\n');

fprintf('Saturation duration: %.4f s\n', ...
    saturation_duration);

fprintf('Saturation percentage: %.2f %%\n', ...
    saturation_percentage);

fprintf('============================================\n');

%% ============================================================
% ANIMATION SETTINGS
% ============================================================

animation_speed = 2.5;

frame_dt = 0.04;

%% Interpolation time

t_anim = t_start:frame_dt:t_end;

X_anim = interp1( ...
    t, ...
    X, ...
    t_anim, ...
    'linear');

xdes_anim = interp1( ...
    t, ...
    x_des_log, ...
    t_anim);

ydes_anim = interp1( ...
    t, ...
    y_des_log, ...
    t_anim);

zdes_anim = interp1( ...
    t, ...
    z_des_log, ...
    t_anim);

error_anim = interp1( ...
    t, ...
    tracking_error, ...
    t_anim);

motor_anim = interp1( ...
    t, ...
    motor_log, ...
    t_anim);

%% ============================================================
% FIGURE
% ============================================================

fig = figure( ...
    'Name','AeroStabilize - LQR Wind Disturbance', ...
    'NumberTitle','off');

% Smaller animation window
fig.Position = [ ...
    150, ...
    100, ...
    1200, ...
    700];

tl = tiledlayout( ...
    fig, ...
    2, ...
    2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% ============================================================
% 3D VIEW
% ============================================================

ax3d = nexttile(tl,[2 1]);

hold(ax3d,'on');
grid(ax3d,'on');
axis(ax3d,'equal');

xlabel(ax3d,'X [m]');
ylabel(ax3d,'Y [m]');
zlabel(ax3d,'Z [m]');

title(ax3d,'LQR + Feedforward | 5 N Wind Disturbance');

view(ax3d,38,25);

%% Desired trajectory

plot3( ...
    ax3d, ...
    x_des_log, ...
    y_des_log, ...
    z_des_log, ...
    '--', ...
    'LineWidth',1.4);

%% Actual trajectory handle

actual_path = plot3( ...
    ax3d, ...
    NaN, ...
    NaN, ...
    NaN, ...
    'LineWidth',2.0);

%% Desired current position

desired_point = plot3( ...
    ax3d, ...
    NaN, ...
    NaN, ...
    NaN, ...
    'o', ...
    'MarkerSize',8, ...
    'LineWidth',1.5);

%% Drone arms

arm1 = plot3( ...
    ax3d, ...
    NaN,NaN,NaN, ...
    'LineWidth',4);

arm2 = plot3( ...
    ax3d, ...
    NaN,NaN,NaN, ...
    'LineWidth',4);

%% Drone body center

body_point = plot3( ...
    ax3d, ...
    NaN,NaN,NaN, ...
    'o', ...
    'MarkerSize',8, ...
    'MarkerFaceColor','auto');

%% Rotor-circle handles

rotor1 = plot3(ax3d,NaN,NaN,NaN,'LineWidth',1.4);
rotor2 = plot3(ax3d,NaN,NaN,NaN,'LineWidth',1.4);
rotor3 = plot3(ax3d,NaN,NaN,NaN,'LineWidth',1.4);
rotor4 = plot3(ax3d,NaN,NaN,NaN,'LineWidth',1.4);

%% Wind arrow

wind_arrow = quiver3( ...
    ax3d, ...
    0,0,0, ...
    0,0,0, ...
    0, ...
    'LineWidth',3, ...
    'MaxHeadSize',1.5);

%% Status text

status_text = text( ...
    ax3d, ...
    0.02, ...
    0.98, ...
    '', ...
    'Units','normalized', ...
    'VerticalAlignment','top', ...
    'FontName','Consolas', ...
    'FontSize',10);

%% Axis limits

xmin = min(x_des_log)-1.5;
xmax = max(x_des_log)+1.5;

ymin = min(y_des_log)-1.5;
ymax = max(y_des_log)+1.5;

zmin = 0;
zmax = max(z_des_log)+1.5;

xlim(ax3d,[xmin xmax]);
ylim(ax3d,[ymin ymax]);
zlim(ax3d,[zmin zmax]);

%% ============================================================
% TRACKING ERROR PANEL
% ============================================================

ax_error = nexttile(tl);

hold(ax_error,'on');
grid(ax_error,'on');

plot( ...
    ax_error, ...
    t, ...
    tracking_error, ...
    'LineWidth',1.5);

xline( ...
    ax_error, ...
    wind_start, ...
    '--', ...
    'Wind ON');

xline( ...
    ax_error, ...
    wind_end, ...
    '--', ...
    'Wind OFF');

error_marker = plot( ...
    ax_error, ...
    NaN, ...
    NaN, ...
    'o', ...
    'MarkerSize',7, ...
    'MarkerFaceColor','auto');

xlabel(ax_error,'Time [s]');
ylabel(ax_error,'3D Error [m]');

title(ax_error,'Position Tracking Error');

xlim(ax_error,[t_start t_end]);

%% ============================================================
% MOTOR THRUST PANEL
% ============================================================

ax_motor = nexttile(tl);

motor_bar = bar( ...
    ax_motor, ...
    1:4, ...
    zeros(1,4));

grid(ax_motor,'on');

xlabel(ax_motor,'Motor');
ylabel(ax_motor,'Thrust [N]');

title(ax_motor,'Individual Motor Thrust');

ylim(ax_motor,[0 T_motor_max+0.5]);

xticks(ax_motor,1:4);

yline( ...
    ax_motor, ...
    T_motor_max, ...
    '--', ...
    'Limit');

%% ============================================================
% DRONE GEOMETRY
% ============================================================

arm_length_visual = 0.30;

rotor_radius = 0.09;

rotor_angle = linspace(0,2*pi,30);

%% Plus configuration rotor centers in body frame

rotor_centers_body = [ ...
     arm_length_visual,  0,                   0;
     0,                  arm_length_visual,   0;
    -arm_length_visual,  0,                   0;
     0,                 -arm_length_visual,   0]';

%% Arm endpoints

arm_x_body = [ ...
    -arm_length_visual, ...
     arm_length_visual;
     0, ...
     0;
     0, ...
     0];

arm_y_body = [ ...
     0, ...
     0;
    -arm_length_visual, ...
     arm_length_visual;
     0, ...
     0];

%% ============================================================
% ANIMATION LOOP
% ============================================================

for k = 1:length(t_anim)

    if ~isgraphics(fig)
        break;
    end

    tk = t_anim(k);

    state = X_anim(k,:)';

    pos = state(1:3);

    phi_k   = state(7);
    theta_k = state(8);
    psi_k   = state(9);

    %% Rotation matrices

    Rx = [ ...
        1, 0, 0;
        0, cos(phi_k), -sin(phi_k);
        0, sin(phi_k),  cos(phi_k)];

    Ry = [ ...
         cos(theta_k), 0, sin(theta_k);
         0,            1, 0;
        -sin(theta_k), 0, cos(theta_k)];

    Rz = [ ...
        cos(psi_k), -sin(psi_k), 0;
        sin(psi_k),  cos(psi_k), 0;
        0,           0,          1];

    Rbw = Rz*Ry*Rx;

    %% Actual path

    if isgraphics(actual_path)

        set( ...
            actual_path, ...
            'XData',X_anim(1:k,1), ...
            'YData',X_anim(1:k,2), ...
            'ZData',X_anim(1:k,3));

    end

    %% Desired point

    if isgraphics(desired_point)

        set( ...
            desired_point, ...
            'XData',xdes_anim(k), ...
            'YData',ydes_anim(k), ...
            'ZData',zdes_anim(k));

    end

    %% Drone arms

    arm1_world = Rbw*arm_x_body + pos;
    arm2_world = Rbw*arm_y_body + pos;

    if isgraphics(arm1)

        set( ...
            arm1, ...
            'XData',arm1_world(1,:), ...
            'YData',arm1_world(2,:), ...
            'ZData',arm1_world(3,:));

    end

    if isgraphics(arm2)

        set( ...
            arm2, ...
            'XData',arm2_world(1,:), ...
            'YData',arm2_world(2,:), ...
            'ZData',arm2_world(3,:));

    end

    %% Body center

    if isgraphics(body_point)

        set( ...
            body_point, ...
            'XData',pos(1), ...
            'YData',pos(2), ...
            'ZData',pos(3));

    end

    %% Rotor circles

    rotor_handles = [ ...
        rotor1, ...
        rotor2, ...
        rotor3, ...
        rotor4];

    for rotor_idx = 1:4

        rotor_center_world = ...
            Rbw*rotor_centers_body(:,rotor_idx) + pos;

        rotor_circle_body = [ ...
            rotor_radius*cos(rotor_angle);
            rotor_radius*sin(rotor_angle);
            zeros(size(rotor_angle))];

        rotor_circle_world = ...
            Rbw*rotor_circle_body + rotor_center_world;

        if isgraphics(rotor_handles(rotor_idx))

            set( ...
                rotor_handles(rotor_idx), ...
                'XData',rotor_circle_world(1,:), ...
                'YData',rotor_circle_world(2,:), ...
                'ZData',rotor_circle_world(3,:));

        end

    end

    %% Wind arrow

    wind_active = ...
        tk >= wind_start && ...
        tk <= wind_end;

    if isgraphics(wind_arrow)

        if wind_active

            arrow_scale = 0.12;

            set( ...
                wind_arrow, ...
                'XData',pos(1), ...
                'YData',pos(2), ...
                'ZData',pos(3)+0.3, ...
                'UData',F_wind(1)*arrow_scale, ...
                'VData',F_wind(2)*arrow_scale, ...
                'WData',F_wind(3)*arrow_scale);

        else

            set( ...
                wind_arrow, ...
                'XData',pos(1), ...
                'YData',pos(2), ...
                'ZData',pos(3)+0.3, ...
                'UData',0, ...
                'VData',0, ...
                'WData',0);

        end

    end

    %% Error marker

    if isgraphics(error_marker)

        set( ...
            error_marker, ...
            'XData',tk, ...
            'YData',error_anim(k));

    end

    %% Motor bars

    if isgraphics(motor_bar)

        motor_bar.YData = motor_anim(k,:);

    end

    %% Status display

    if wind_active
        wind_status = 'ON (+5 N X)';
    else
        wind_status = 'OFF';
    end

    status_string = sprintf([ ...
        'Time:       %5.2f s\n' ...
        'Controller: LQR + FF\n' ...
        'Error:      %6.3f m\n' ...
        'Roll:       %6.2f deg\n' ...
        'Pitch:      %6.2f deg\n' ...
        'Wind:       %s'], ...
        tk, ...
        error_anim(k), ...
        rad2deg(phi_k), ...
        rad2deg(theta_k), ...
        wind_status);

    if isgraphics(status_text)

        set( ...
            status_text, ...
            'String',status_string);

    end

    drawnow limitrate;

    pause(frame_dt/animation_speed);

end

%% ============================================================
% LOCAL FUNCTION: HELIX REFERENCE
% ============================================================

function [ ...
    x_des, ...
    y_des, ...
    z_des, ...
    vx_des, ...
    vy_des, ...
    vz_des, ...
    ax_ff, ...
    ay_ff, ...
    az_ff] = helix_reference( ...
    t, ...
    R, ...
    omega, ...
    z0, ...
    vz_const)

x_des = R*cos(omega*t);

y_des = R*sin(omega*t);

z_des = z0 + vz_const*t;

vx_des = -R*omega*sin(omega*t);

vy_des =  R*omega*cos(omega*t);

vz_des = vz_const;

ax_ff = -R*omega^2*cos(omega*t);

ay_ff = -R*omega^2*sin(omega*t);

az_ff = 0;

end

%% ============================================================
% LOCAL FUNCTION: NONLINEAR QUADCOPTER DYNAMICS
% ============================================================

function dx = quad_lqr_wind_dynamics( ...
    t, ...
    state, ...
    K, ...
    m, ...
    g, ...
    I, ...
    L, ...
    k_yaw, ...
    T_motor_min, ...
    T_motor_max, ...
    R_helix, ...
    omega_helix, ...
    z0_helix, ...
    vz_helix, ...
    wind_start, ...
    wind_end, ...
    F_wind)

%% Extract states

vx = state(4);
vy = state(5);
vz = state(6);

phi   = state(7);
theta = state(8);
psi   = state(9);

p = state(10);
q = state(11);
r = state(12);

%% ============================================================
% TRAJECTORY REFERENCE
% ============================================================

[xd,yd,zd, ...
 vxd,vyd,vzd, ...
 ax_ff,ay_ff,az_ff] = helix_reference( ...
    t, ...
    R_helix, ...
    omega_helix, ...
    z0_helix, ...
    vz_helix);

%% Feedforward attitude

theta_ff = ax_ff/g;
phi_ff   = -ay_ff/g;

%% Desired state

x_ref = zeros(12,1);

x_ref(1) = xd;
x_ref(2) = yd;
x_ref(3) = zd;

x_ref(4) = vxd;
x_ref(5) = vyd;
x_ref(6) = vzd;

x_ref(7) = phi_ff;
x_ref(8) = theta_ff;
x_ref(9) = 0;

%% ============================================================
% LQR CONTROL
% ============================================================

e = state - x_ref;

delta_u = -K*e;

%% Feedforward thrust

T_ff = m*(g + az_ff);

u_cmd = [ ...
    T_ff;
    0;
    0;
    0] + delta_u;

%% ============================================================
% CONTROL ALLOCATION
% ============================================================

M = [ ...
    1,      1,      1,      1;
    0,      L,      0,     -L;
   -L,      0,      L,      0;
    k_yaw, -k_yaw,  k_yaw, -k_yaw];

motor_thrust = M\u_cmd;

%% Motor saturation

motor_thrust = min( ...
    max(motor_thrust,T_motor_min), ...
    T_motor_max);

%% Achievable generalized controls

u_actual = M*motor_thrust;

T = u_actual(1);

tau = u_actual(2:4);

%% ============================================================
% BODY-TO-WORLD ROTATION
% ============================================================

Rx = [ ...
    1, 0, 0;
    0, cos(phi), -sin(phi);
    0, sin(phi),  cos(phi)];

Ry = [ ...
     cos(theta), 0, sin(theta);
     0,          1, 0;
    -sin(theta), 0, cos(theta)];

Rz = [ ...
    cos(psi), -sin(psi), 0;
    sin(psi),  cos(psi), 0;
    0,         0,        1];

Rbw = Rz*Ry*Rx;

%% ============================================================
% TRANSLATIONAL DYNAMICS
% ============================================================

F_thrust_body = [ ...
    0;
    0;
    T];

F_thrust_world = ...
    Rbw*F_thrust_body;

F_gravity = [ ...
    0;
    0;
    -m*g];

%% Wind

if t >= wind_start && t <= wind_end

    F_external = F_wind;

else

    F_external = [ ...
        0;
        0;
        0];

end

acc = ( ...
    F_thrust_world + ...
    F_gravity + ...
    F_external) / m;

%% ============================================================
% ROTATIONAL DYNAMICS
% ============================================================

omega_body = [ ...
    p;
    q;
    r];

omega_dot = I \ ( ...
    tau - ...
    cross(omega_body,I*omega_body));

%% ============================================================
% EULER KINEMATICS
% ============================================================

E = [ ...
    1, ...
    sin(phi)*tan(theta), ...
    cos(phi)*tan(theta);
    0, ...
    cos(phi), ...
    -sin(phi);
    0, ...
    sin(phi)/cos(theta), ...
    cos(phi)/cos(theta)];

euler_dot = E*omega_body;

%% ============================================================
% STATE DERIVATIVE
% ============================================================

dx = zeros(12,1);

dx(1) = vx;
dx(2) = vy;
dx(3) = vz;

dx(4:6) = acc;

dx(7:9) = euler_dot;

dx(10:12) = omega_dot;

end