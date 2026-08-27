%% AeroStabilize - Quadcopter Parameters

% Environment
g = 9.81;

% Vehicle
m = 1.5;
L = 0.225;

% Moments of inertia
Ix = 0.02;
Iy = 0.02;
Iz = 0.04;

I = diag([Ix Iy Iz]);

% Yaw torque coefficient
k_yaw = 0.02;