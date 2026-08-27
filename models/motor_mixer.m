function motor_thrusts = motor_mixer(T, tau_phi, tau_theta, tau_psi, L, k_yaw)

% AeroStabilize - Motor Mixer
%
% Plus configuration:
%
%            Motor 1
%               ^
%               |
%   Motor 4 <-- + --> Motor 2
%               |
%               v
%            Motor 3
%
% Inputs:
% T         = Total thrust [N]
% tau_phi   = Roll torque [N*m]
% tau_theta = Pitch torque [N*m]
% tau_psi   = Yaw torque [N*m]
%
% Output:
% motor_thrusts = [T1; T2; T3; T4]

% Mixing matrix
M = [1,  1,  1,  1;
     0,  L,  0, -L;
    -L,  0,  L,  0;
     k_yaw, -k_yaw, k_yaw, -k_yaw];

% Desired control vector
control = [T;
           tau_phi;
           tau_theta;
           tau_psi];

% Solve for individual motor thrusts
motor_thrusts = M \ control;

end