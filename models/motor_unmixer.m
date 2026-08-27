function u = motor_unmixer(motor_thrusts, L, k_yaw)

% AeroStabilize - Motor Unmixer
%
% motor_thrusts = [T1; T2; T3; T4]
%
% Output:
% u = [T; tau_phi; tau_theta; tau_psi]

T1 = motor_thrusts(1);
T2 = motor_thrusts(2);
T3 = motor_thrusts(3);
T4 = motor_thrusts(4);

% Total thrust
T = T1 + T2 + T3 + T4;

% Roll torque
tau_phi = L * (T2 - T4);

% Pitch torque
tau_theta = L * (T3 - T1);

% Yaw torque
tau_psi = k_yaw * (T1 - T2 + T3 - T4);

u = [T;
     tau_phi;
     tau_theta;
     tau_psi];

end