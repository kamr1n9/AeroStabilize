%% AeroStabilize
% 3-Axis LQI Validation
%
% Tests:
%   1) Position regulation
%   2) Constant lateral disturbance
%   3) Mid-flight +20% payload increase
%
% Goal:
% Demonstrate rejection of constant disturbances and model mismatch
% using integral augmentation in x, y and z.

clear;
clc;
close all;

%% ============================================================
% PARAMETERS
% ============================================================

g = 9.81;

m_nominal = 1.5;     % controller model [kg]
m_payload = 1.8;     % actual mass after payload change [kg]

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
% LINEARIZED 12-STATE MODEL
% ============================================================

A = zeros(12);

% Position kinematics
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

% Small-angle translational coupling
A(4,8) = g;
A(5,7) = -g;

% Euler-angle kinematics
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

B = zeros(12,4);

% Total thrust -> vertical acceleration
B(6,1) = 1/m_nominal;

% Body torques -> angular acceleration
B(10,2) = 1/Ix;
B(11,3) = 1/Iy;
B(12,4) = 1/Iz;

%% ============================================================
% 3-AXIS INTEGRAL AUGMENTATION
% ============================================================

% Position output:
% p = [x; y; z]

C_pos = zeros(3,12);

C_pos(1,1) = 1;
C_pos(2,2) = 1;
C_pos(3,3) = 1;

% Augmented state:
%
% x_aug =
% [ physical 12 states
%   xi_x
%   xi_y
%   xi_z ]
%
% Integral definition:
%
% xi_dot = p_ref - p
%
% Since the state error used in the feedback law is
%
% e = x - x_ref
%
% we have
%
% xi_dot = -C_pos * e

A_aug = zeros(15,15);

A_aug(1:12,1:12) = A;
A_aug(13:15,1:12) = -C_pos;

B_aug = zeros(15,4);
B_aug(1:12,:) = B;

%% ============================================================
% CONTROLLABILITY CHECK
% ============================================================

Co_aug = ctrb(A_aug,B_aug);

rank_aug = rank(Co_aug);

fprintf('============================================\n');
fprintf('3-Axis LQI Controller\n');
fprintf('============================================\n');

fprintf('Augmented controllability rank: %d / 15\n', ...
    rank_aug);

if rank_aug < 15
    warning('Augmented system is not fully controllable.');
end

%% ============================================================
% LQI WEIGHTING MATRICES
% ============================================================

q_weights = [ ...
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
    20, ...   % xi_x
    20, ...   % xi_y
    25];      % xi_z

Q_aug = diag(q_weights);

R = diag([ ...
    1.0, ...
    0.5, ...
    0.5, ...
    0.5]);

%% Dimension checks

fprintf('\nMatrix dimensions:\n');

fprintf('A_aug: %d x %d\n', ...
    size(A_aug,1),size(A_aug,2));

fprintf('B_aug: %d x %d\n', ...
    size(B_aug,1),size(B_aug,2));

fprintf('Q_aug: %d x %d\n', ...
    size(Q_aug,1),size(Q_aug,2));

fprintf('R:     %d x %d\n', ...
    size(R,1),size(R,2));

%% ============================================================
% LQI GAIN
% ============================================================

K_aug = lqr(A_aug,B_aug,Q_aug,R);

Kx = K_aug(:,1:12);
Ki = K_aug(:,13:15);

disp(' ');
disp('Integral gain matrix Ki:');
disp(Ki);

%% ============================================================
% SIMULATION SETTINGS
% ============================================================

t_start = 0;
t_end   = 35;

x0 = zeros(15,1);

%% Desired position

x_des = 1.0;
y_des = 1.0;
z_des = 2.0;

%% ============================================================
% DISTURBANCE SCHEDULE
% ============================================================

% Constant lateral disturbance begins at 10 s
wind_start = 10.0;

% Payload is attached at 20 s
payload_time = 20.0;

% Constant external disturbance
F_wind = [ ...
    2.0;
    0.0;
    0.0];

%% ============================================================
% SIMULATION
% ============================================================

odefun = @(t,x_aug_state) lqi_dynamics( ...
    t, ...
    x_aug_state, ...
    Kx, ...
    Ki, ...
    x_des, ...
    y_des, ...
    z_des, ...
    m_nominal, ...
    m_payload, ...
    payload_time, ...
    wind_start, ...
    F_wind, ...
    g, ...
    I, ...
    L, ...
    k_yaw, ...
    T_motor_min, ...
    T_motor_max);

options = odeset( ...
    'RelTol',1e-7, ...
    'AbsTol',1e-9);

[t,X] = ode45( ...
    odefun, ...
    [t_start t_end], ...
    x0, ...
    options);

%% ============================================================
% EXTRACT STATES
% ============================================================

x = X(:,1);
y = X(:,2);
z = X(:,3);

phi   = X(:,7);
theta = X(:,8);

xi_x = X(:,13);
xi_y = X(:,14);
xi_z = X(:,15);

%% ============================================================
% POSITION ERRORS
% ============================================================

e_x = x_des - x;
e_y = y_des - y;
e_z = z_des - z;

position_error = sqrt( ...
    e_x.^2 + ...
    e_y.^2 + ...
    e_z.^2);

%% ============================================================
% RECONSTRUCT CONTROL HISTORY
% ============================================================

N = length(t);

T_total_log = zeros(N,1);

motor_log = zeros(N,4);

mass_log = zeros(N,1);

saturation_log = false(N,1);

M = [ ...
    1,      1,      1,      1;
    0,      L,      0,     -L;
   -L,      0,      L,      0;
    k_yaw, -k_yaw,  k_yaw, -k_yaw];

for k = 1:N

    tk = t(k);

    state = X(k,1:12)';
    xi    = X(k,13:15)';

    %% Desired state

    x_ref = zeros(12,1);

    x_ref(1) = x_des;
    x_ref(2) = y_des;
    x_ref(3) = z_des;

    %% State error

    e = state - x_ref;

    %% LQI control

    delta_u = -Kx*e - Ki*xi;

    %% Nominal hover feedforward

    u_cmd = [ ...
        m_nominal*g;
        0;
        0;
        0] + delta_u;

    %% Requested motor thrust

    motor_requested = M\u_cmd;

    %% Detect saturation

    saturation_log(k) = any( ...
        motor_requested < T_motor_min | ...
        motor_requested > T_motor_max);

    %% Apply motor limits

    motor_thrust = min( ...
        max(motor_requested,T_motor_min), ...
        T_motor_max);

    %% Achievable generalized control

    u_actual = M*motor_thrust;

    T_total_log(k) = u_actual(1);

    motor_log(k,:) = motor_thrust';

    %% Actual mass

    if tk < payload_time
        mass_log(k) = m_nominal;
    else
        mass_log(k) = m_payload;
    end

end

%% ============================================================
% PERFORMANCE METRICS
% ============================================================

final_error_x = e_x(end);
final_error_y = e_y(end);
final_error_z = e_z(end);

final_position_error = position_error(end);

RMSE = sqrt(mean(position_error.^2));

max_error = max(position_error);

actual_hover_thrust = m_payload*g;

saturation_duration = trapz( ...
    t, ...
    double(saturation_log));

saturation_percentage = ...
    100*saturation_duration/(t(end)-t(1));

%% ============================================================
% RESULTS
% ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('3-Axis LQI Results\n');
fprintf('============================================\n');

fprintf('RMSE: %.4f m\n',RMSE);

fprintf('Maximum position error: %.4f m\n', ...
    max_error);

fprintf('\nFinal position errors:\n');

fprintf('X error: %.6f m\n', ...
    final_error_x);

fprintf('Y error: %.6f m\n', ...
    final_error_y);

fprintf('Z error: %.6f m\n', ...
    final_error_z);

fprintf('\n');

fprintf('Final 3D position error: %.6f m\n', ...
    final_position_error);

fprintf('\nFinal integral states:\n');

fprintf('xi_x = %.6f\n',xi_x(end));
fprintf('xi_y = %.6f\n',xi_y(end));
fprintf('xi_z = %.6f\n',xi_z(end));

fprintf('\n');

fprintf('Final total thrust: %.4f N\n', ...
    T_total_log(end));

fprintf('Required hover thrust at %.1f kg: %.4f N\n', ...
    m_payload, ...
    actual_hover_thrust);

fprintf('\n');

fprintf('Maximum motor thrust: %.4f N\n', ...
    max(motor_log,[],'all'));

fprintf('Minimum motor thrust: %.4f N\n', ...
    min(motor_log,[],'all'));

fprintf('\n');

fprintf('Motor saturation duration: %.4f s\n', ...
    saturation_duration);

fprintf('Motor saturation percentage: %.2f %%\n', ...
    saturation_percentage);

fprintf('============================================\n');

%% ============================================================
% PLOT 1: 3D POSITION
% ============================================================

figure('Position',[150 120 800 550]);

plot3(x,y,z,'LineWidth',1.8);
hold on;

plot3( ...
    x_des, ...
    y_des, ...
    z_des, ...
    'o', ...
    'MarkerSize',9, ...
    'LineWidth',2);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title('3-Axis LQI Position Regulation');

legend( ...
    'Actual trajectory', ...
    'Desired position', ...
    'Location','best');

%% ============================================================
% PLOT 2: POSITION STATES
% ============================================================

figure('Position',[180 140 800 500]);

plot(t,x,'LineWidth',1.5);
hold on;

plot(t,y,'LineWidth',1.5);
plot(t,z,'LineWidth',1.5);

yline(x_des,'--');
yline(y_des,'--');
yline(z_des,'--');

xline(wind_start,'--','Wind ON');
xline(payload_time,'--','Payload +20%');

grid on;

xlabel('Time [s]');
ylabel('Position [m]');

title('LQI Position Response');

legend( ...
    'x','y','z', ...
    'x_d','y_d','z_d', ...
    'Location','best');

%% ============================================================
% PLOT 3: POSITION ERRORS
% ============================================================

figure('Position',[210 160 800 500]);

plot(t,e_x,'LineWidth',1.5);
hold on;

plot(t,e_y,'LineWidth',1.5);
plot(t,e_z,'LineWidth',1.5);

xline(wind_start,'--','Wind ON');
xline(payload_time,'--','Payload +20%');

yline(0,'--');

grid on;

xlabel('Time [s]');
ylabel('Position Error [m]');

title('3-Axis LQI Tracking Errors');

legend( ...
    'e_x', ...
    'e_y', ...
    'e_z', ...
    'Location','best');

%% ============================================================
% PLOT 4: 3D POSITION ERROR
% ============================================================

figure('Position',[240 180 800 500]);

plot(t,position_error,'LineWidth',1.7);

hold on;

xline(wind_start,'--','Wind ON');
xline(payload_time,'--','Payload +20%');

grid on;

xlabel('Time [s]');
ylabel('3D Position Error [m]');

title('LQI Position Error Magnitude');

%% ============================================================
% PLOT 5: INTEGRAL STATES
% ============================================================

figure('Position',[270 200 800 500]);

plot(t,xi_x,'LineWidth',1.5);
hold on;

plot(t,xi_y,'LineWidth',1.5);
plot(t,xi_z,'LineWidth',1.5);

xline(wind_start,'--','Wind ON');
xline(payload_time,'--','Payload +20%');

grid on;

xlabel('Time [s]');
ylabel('Integral State');

title('LQI Integral States');

legend( ...
    '\xi_x', ...
    '\xi_y', ...
    '\xi_z', ...
    'Location','best');

%% ============================================================
% PLOT 6: TOTAL THRUST
% ============================================================

figure('Position',[300 220 800 500]);

plot(t,T_total_log,'LineWidth',1.7);

hold on;

yline( ...
    m_nominal*g, ...
    '--', ...
    'Nominal Hover Thrust');

yline( ...
    m_payload*g, ...
    '--', ...
    'Payload Hover Thrust');

xline(wind_start,'--','Wind ON');
xline(payload_time,'--','Payload +20%');

grid on;

xlabel('Time [s]');
ylabel('Total Thrust [N]');

title('LQI Total Thrust');

%% ============================================================
% PLOT 7: MOTOR THRUST
% ============================================================

figure('Position',[330 240 800 500]);

plot(t,motor_log(:,1),'LineWidth',1.3);
hold on;

plot(t,motor_log(:,2),'LineWidth',1.3);
plot(t,motor_log(:,3),'LineWidth',1.3);
plot(t,motor_log(:,4),'LineWidth',1.3);

yline(T_motor_max,'--','Motor Limit');

xline(wind_start,'--','Wind ON');
xline(payload_time,'--','Payload +20%');

grid on;

xlabel('Time [s]');
ylabel('Motor Thrust [N]');

title('Individual Motor Thrusts');

legend( ...
    'Motor 1', ...
    'Motor 2', ...
    'Motor 3', ...
    'Motor 4', ...
    'Location','best');

%% ============================================================
% LOCAL FUNCTION
% ============================================================

function dx_aug = lqi_dynamics( ...
    t, ...
    x_aug, ...
    Kx, ...
    Ki, ...
    x_des, ...
    y_des, ...
    z_des, ...
    m_nominal, ...
    m_payload, ...
    payload_time, ...
    wind_start, ...
    F_wind, ...
    g, ...
    I, ...
    L, ...
    k_yaw, ...
    T_motor_min, ...
    T_motor_max)

%% Extract states

x = x_aug(1);
y = x_aug(2);
z = x_aug(3);

vx = x_aug(4);
vy = x_aug(5);
vz = x_aug(6);

phi   = x_aug(7);
theta = x_aug(8);
psi   = x_aug(9);

p = x_aug(10);
q = x_aug(11);
r = x_aug(12);

xi = x_aug(13:15);

state = x_aug(1:12);

%% Actual mass

if t < payload_time
    m_actual = m_nominal;
else
    m_actual = m_payload;
end

%% Desired state

x_ref = zeros(12,1);

x_ref(1) = x_des;
x_ref(2) = y_des;
x_ref(3) = z_des;

%% State error

e = state - x_ref;

%% LQI control

delta_u = -Kx*e - Ki*xi;

u_cmd = [ ...
    m_nominal*g;
    0;
    0;
    0] + delta_u;

%% Mixer

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

%% Achievable thrust and torques

u_actual = M*motor_thrust;

T = u_actual(1);

tau = u_actual(2:4);

%% ============================================================
% ROTATION MATRIX
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

F_thrust_body = [0;0;T];

F_thrust_world = ...
    Rbw*F_thrust_body;

F_gravity = [ ...
    0;
    0;
    -m_actual*g];

%% Constant wind after wind_start

if t >= wind_start
    F_external = F_wind;
else
    F_external = [0;0;0];
end

acc = ( ...
    F_thrust_world + ...
    F_gravity + ...
    F_external) / m_actual;

%% ============================================================
% ROTATIONAL DYNAMICS
% ============================================================

omega = [p;q;r];

omega_dot = I \ ( ...
    tau - ...
    cross(omega,I*omega));

%% ============================================================
% EULER KINEMATICS
% ============================================================

E = [ ...
    1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi),           -sin(phi);
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

euler_dot = E*omega;

%% ============================================================
% PHYSICAL STATE DERIVATIVE
% ============================================================

dx = zeros(12,1);

dx(1) = vx;
dx(2) = vy;
dx(3) = vz;

dx(4:6) = acc;

dx(7:9) = euler_dot;

dx(10:12) = omega_dot;

%% ============================================================
% INTEGRAL STATES
% ============================================================

xi_dot = [ ...
    x_des - x;
    y_des - y;
    z_des - z];

%% Complete augmented derivative

dx_aug = [ ...
    dx;
    xi_dot];

end

