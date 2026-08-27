function xdot = quad_dynamics(x, u, m, g, I)

%state vector
% x = [x y z vx vy vz phi theta psi p q r]'

%inputs
% u = [T tau_phi tau_theta tau_psi]'

%extract states
vx    = x(4);
vy    = x(5);
vz    = x(6);

phi   = x(7);
theta = x(8);
psi   = x(9);

p     = x(10);
q     = x(11);
r     = x(12);

%extract inputs
T         = u(1);
tau_phi   = u(2);
tau_theta = u(3);
tau_psi   = u(4);

tau = [tau_phi; tau_theta; tau_psi];

%% Rotation matrix: Body frame -> World frame

R_x = [1 0 0;
       0 cos(phi) -sin(phi);
       0 sin(phi)  cos(phi)];

R_y = [ cos(theta) 0 sin(theta);
        0          1 0;
       -sin(theta) 0 cos(theta)];

R_z = [cos(psi) -sin(psi) 0;
       sin(psi)  cos(psi) 0;
       0         0        1];

R = R_z * R_y * R_x;

%% Translational dynamics

% Thrust force in body frame
F_thrust_body = [0; 0; T];

% Transform thrust into world frame
F_thrust_world = R * F_thrust_body;

% Gravity force
F_gravity = [0; 0; -m*g];

% Total acceleration
acc = (F_thrust_world + F_gravity) / m;
%% Rotational dynamics

omega = [p; q; r];

omega_dot = I \ (tau - cross(omega, I * omega));

%% Euler angle rates

E = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
     0, cos(phi),            -sin(phi);
     0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];

euler_dot = E * omega;

%state derivative vector
xdot = zeros(12,1);

% Position derivatives
xdot(1) = vx;
xdot(2) = vy;
xdot(3) = vz;

% Velocity derivatives
xdot(4) = acc(1);
xdot(5) = acc(2);
xdot(6) = acc(3);

% Euler angle derivatives
xdot(7) = euler_dot(1);
xdot(8) = euler_dot(2);
xdot(9) = euler_dot(3);

% Angular rate derivatives
xdot(10) = omega_dot(1);
xdot(11) = omega_dot(2);
xdot(12) = omega_dot(3);

end