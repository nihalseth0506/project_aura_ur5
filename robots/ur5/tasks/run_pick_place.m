function [total_energy, tau_history, power_history] = run_pick_place(object_mass)

addpath(genpath(pwd));

%% Initial configuration
q = [0; -pi/4; pi/3; -pi/6; pi/4; -pi/2];

[~,~,~,~,~,T] = forward_chain_ur5(q');
x_home = T(1:3,4);

waypoints = generate_pick_place_waypoints(x_home);

x_pick  = waypoints(:,2);
x_place = waypoints(:,5);

offset = [0;0;0.1];

task_sequence = {
    x_pick + offset,          "approach_pick";
    x_pick + [0;0;0.02],      "lower_pick";
    x_pick + [0;0;0.02],      "grip";
    x_pick + offset,          "lift";
    x_place + offset,         "approach_place";
    x_place + [0;0;0.02],     "lower_place";
    x_place + [0;0;0.02],     "release";
    x_place + offset,         "lift_after_place";
    x_home,                   "return_home"
};

dt = 0.02;
K  = 3;

payload_mass = 0;

tau_history = [];
power_history = [];
qdot_prev = zeros(6,1);

for i = 1:size(task_sequence,1)

    xd_target = task_sequence{i,1};
    task_phase = task_sequence{i,2};

    if strcmp(task_phase,"grip")
        payload_mass = object_mass;
    end

    if strcmp(task_phase,"release")
        payload_mass = 0;
    end

    [~,~,~,~,~,T] = forward_chain_ur5(q');
    x_current = T(1:3,4);

    X_segment = interpolate_cartesian_segment(x_current, xd_target, 40);

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

        s = svd(J);
        w = prod(s);

        lambda = 0.01 + 5e-5/(w + 1e-6);
        lambda = min(lambda,0.3);

        qdot = J' * ((J*J' + lambda^2 * eye(6)) \ [xdot;0;0;0]);
        qdot = apply_joint_velocity_limits(qdot);

        q = q + qdot*dt;
        q = apply_joint_limits(q);

        qddot = (qdot - qdot_prev)/dt;
        qdot_prev = qdot;

        tau = inverse_dynamics_with_payload_ur5(q, qdot, qddot, payload_mass);

        tau_max = [150;150;100;80;50;50];
        tau = max(min(tau, tau_max), -tau_max);

        power = abs(tau' * qdot);  % absolute energy

        tau_history = [tau_history tau];
        power_history = [power_history power];

    end
end

total_energy = sum(power_history)*dt;

end