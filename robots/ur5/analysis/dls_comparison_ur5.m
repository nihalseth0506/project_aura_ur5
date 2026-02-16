clc;
clear;
close all;

addpath(genpath(pwd))

% Near-singular configuration
theta_vals = [0, 0, 0, 0, 0, 0];

J = jacobian_ur5(theta_vals);

% Desired Cartesian velocity (move in +X)
xdot = [0.05; 0; 0; 0; 0; 0];

fprintf("Condition number: %f\n", cond(J));

%% ---- Pure Inverse (if possible) ----
if rank(J) == 6
    qdot_inv = inv(J) * xdot;
    fprintf("Pure inverse joint velocity norm: %f\n", norm(qdot_inv));
else
    fprintf("Pure inverse undefined (singular configuration)\n");
end

%% ---- Damped Least Squares ----
lambda_values = [0.01 0.1 0.5 1];

for k = 1:length(lambda_values)
    
    lambda = lambda_values(k);
    
    qdot_dls = J' * inv(J*J' + lambda^2 * eye(6)) * xdot;
    
    fprintf("Lambda = %.2f → DLS joint velocity norm: %f\n", ...
            lambda, norm(qdot_dls));
end
