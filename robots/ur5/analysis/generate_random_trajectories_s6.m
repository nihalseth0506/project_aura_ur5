function trajectories = generate_random_trajectories_s6(start_pos, goal_pos, num_paths)

dt = 0.02;
T = 20;
t = (0:dt:T)';
tau = t / T;

trajectories = cell(num_paths,1);

for i = 1:num_paths

    %% Random mid point (introduces variation)
    mid = (start_pos + goal_pos)/2 + 0.2*(rand(1,3)-0.5);

    %% Quadratic Bezier curve
    traj = (1 - tau).^2 .* start_pos + ...
           2*(1 - tau).*tau .* mid + ...
           tau.^2 .* goal_pos;

    %% Store
    trajectories{i}.type = sprintf('traj_%d',i);
    trajectories{i}.points = traj;
    trajectories{i}.xd = traj;
    trajectories{i}.xd_dot = [zeros(1,3); diff(traj)/dt];
    trajectories{i}.control = mid;
end

end