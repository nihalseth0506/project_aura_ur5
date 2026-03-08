clc;
clear;
close all;
addpath(genpath(pwd));

%% -------------------------------
% Initial configuration
%% -------------------------------
q = [0; -pi/4; pi/3; -pi/6; pi/4; -pi/2];
%q = [0; -pi/4; pi/3; 0; 0; 0];

[~,~,~,~,~,T] = forward_chain_ur5(q');
x_home = T(1:3,4);

%% -------------------------------
% Base pick/place positions
%% -------------------------------
waypoints = generate_pick_place_waypoints(x_home);

x_pick  = waypoints(:,2);
x_place = waypoints(:,5);

offset = [0;0;0.1];

payload_flag = [];

%% -------------------------------
% Task State Machine
%% -------------------------------

task_sequence = {
    x_pick + offset,  "approach_pick";
    x_pick + [0;0;0.02], "lower_pick";
    x_pick + [0;0;0.02],  "grip";            % stay at bottom
    x_pick + offset,  "lift";
    x_place + offset, "approach_place";
    x_place + [0;0;0.02],          "lower_place";
    x_place + [0;0;0.02],          "release";         % stay at bottom
    x_place + offset, "lift_after_place";
    x_home,           "return_home"
};

%% -------------------------------
% Simulation parameters
%% -------------------------------
dt = 0.02;
K  = 3;

payload_mass = 0;
object_mass  = 5;  % kg example

q_traj = [];
tau_history = [];
power_history = [];
qdot_prev = zeros(6,1);

%% -------------------------------
% Task Execution Loop
%% -------------------------------
for i = 1:size(task_sequence,1)
    
    xd_target = task_sequence{i,1};
    task_phase = task_sequence{i,2};

    % Payload switching BEFORE motion
    if strcmp(task_phase,"grip")
        payload_mass = object_mass;   % grasp after reaching pick
    end

    if strcmp(task_phase,"release")
        payload_mass = 0;             % release after placing
    end

    % Get current pose
    [~,~,~,~,~,T] = forward_chain_ur5(q');
    x_current = T(1:3,4);

    % Interpolate segment
    X_segment = interpolate_cartesian_segment( ...
        x_current, ...
        xd_target, ...
        40);

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

        % Adaptive damping
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

        % Integrate
        q = q + qdot*dt;
        q = apply_joint_limits(q);

        % Acceleration
        qddot = (qdot - qdot_prev)/dt;
        qdot_prev = qdot;

        % Inverse dynamics WITH payload
        tau = inverse_dynamics_with_payload_ur5(q, qdot, qddot, payload_mass);

        tau_max = [150;150;100;80;50;50];
        tau = max(min(tau, tau_max), -tau_max);

        power = tau' * qdot;

        tau_history = [tau_history tau];
        power_history = [power_history power];

        q_traj = [q_traj q];
        payload_flag = [payload_flag payload_mass > 0];

    end

    % Pause between phases
    for hold = 1:10

    qdot = zeros(6,1);
    qddot = zeros(6,1);

    tau = inverse_dynamics_with_payload_ur5(q, qdot, qddot, payload_mass);

    tau_max = [150;150;100;80;50;50];
    tau = max(min(tau, tau_max), -tau_max);

    power = tau' * qdot;

    tau_history = [tau_history tau];
    power_history = [power_history power];

    q_traj = [q_traj q];
    payload_flag = [payload_flag payload_mass > 0];

end
end

%% -------------------------------
% Energy result
%% -------------------------------
total_energy = sum(power_history)*dt;
disp("Total Task Energy:");
disp(total_energy);

%% -------------------------------
% Plot torque norm
%% -------------------------------
figure;
plot(vecnorm(tau_history),'LineWidth',2);
title('Torque Norm During Pick and Place');
xlabel('Step');
ylabel('||\tau||');
grid on;

hold on;

% approximate indices
pick_index = find(diff(payload_flag)==1,1);
place_index = find(diff(payload_flag)==-1,1,'first');

xline(pick_index,'r--','LineWidth',2);
xline(place_index,'g--','LineWidth',2);

legend('||\tau||','Pick','Place');

figure;
plot(tau_history','LineWidth',1.2);
title('Individual Joint Torques During Pick & Place');
xlabel('Step');
ylabel('Torque (Nm)');
legend('\tau_1','\tau_2','\tau_3','\tau_4','\tau_5','\tau_6');
grid on;

% approximate indices
pick_index = find(diff(payload_flag)==1,1);
place_index = find(diff(payload_flag)==-1,1,'first');

xline(pick_index,'r--','LineWidth',2);
xline(place_index,'g--','LineWidth',2);

%% -------------------------------
% Animate
%% -------------------------------

%animate_ur5_stick(q_traj, waypoints, payload_flag);
