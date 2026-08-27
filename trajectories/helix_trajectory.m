function [x_des, y_des, z_des, ...
          vx_des, vy_des, vz_des, ...
          ax_ff, ay_ff, az_ff] = helix_trajectory(t)

% Helix parameters
R = 2.0;           % Radius [m]
omega = 0.4;       % Angular speed [rad/s]
z0 = 1.0;          % Initial altitude [m]
vz_const = 0.15;   % Vertical climb speed [m/s]

% Desired position
x_des = R * cos(omega * t);
y_des = R * sin(omega * t);
z_des = z0 + vz_const * t;

% Desired velocity
vx_des = -R * omega * sin(omega * t);
vy_des =  R * omega * cos(omega * t);
vz_des = vz_const;

% Desired acceleration
ax_ff = -R * omega^2 * cos(omega * t);
ay_ff = -R * omega^2 * sin(omega * t);
az_ff = 0;

end