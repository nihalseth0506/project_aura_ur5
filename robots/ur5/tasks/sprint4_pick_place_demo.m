clc;
clear;
close all;
addpath(genpath(pwd));

% Initial configuration
q = [0; -pi/4; pi/3; -pi/6; pi/4; 0];

[~,~,~,~,~,T] = forward_chain_ur5(q');
x_home = T(1:3,4);

% Generate waypoints
waypoints = generate_pick_place_waypoints(x_home);

q_traj = [];

dt = 0.02;
K  = 3;

tau_history = [];
power_history = [];
qdot_prev = zeros(6,1);

for i = 1:size(waypoints,2)-1
    
    X_segment = interpolate_cartesian_segment( ...
        waypoints(:,i), ...
        waypoints(:,i+1), ...
        100);
    
    for k = 1:size(X_segment,2)-1
        
        xd      = X_segment(:,k);
        xd_next = X_segment(:,k+1);
    
        [~,~,~,~,~,T] = forward_chain_ur5(q');
        x = T(1:3,4);
    
        e = xd - x;
    
        xdot_ff = (xd_next - xd)/dt;
        xdot_fb = K * e;
        xdot    = xdot_ff + xdot_fb;

        J = jacobian_ur5(q');
        
        % adaptive damping here...

        s = svd(J);
        w = prod(s);

        lambda_min = 0.01;
        lambda_max = 0.3;
        epsilon    = 1e-6;
        k_damp     = 5e-5;

        lambda = lambda_min + k_damp/(w + epsilon);
        lambda = min(lambda, lambda_max);

        qdot = J' * ((J*J' + lambda^2 * eye(6)) \ [xdot;0;0;0]);

        qdot = apply_joint_velocity_limits(qdot);

        q = q + qdot*dt;
        q = apply_joint_limits(q);

        qddot = (qdot - qdot_prev)/dt;
        qdot_prev = qdot;

        tau = inverse_dynamics_ur5(q, qdot, qddot);

        tau_max = [150;150;100;80;50;50];
        tau = max(min(tau, tau_max), -tau_max);
    
        power = tau' * qdot;

        tau_history = [tau_history tau];
        power_history = [power_history power];
        
        q_traj = [q_traj q];
    end
    % Pause at waypoint
    for hold = 1:30
        q_traj = [q_traj q];
    end
end

total_energy = sum(power_history)*dt;
disp("Total Task Energy:");
disp(total_energy);

% Animate
animate_ur5_stick(q_traj,waypoints);