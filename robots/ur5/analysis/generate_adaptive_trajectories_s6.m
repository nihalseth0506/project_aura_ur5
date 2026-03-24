function trajectories = generate_adaptive_trajectories_s6(start_pos,goal_pos,best_traj,num_paths)

dt = 0.02;
T = 20;
t = (0:dt:T)';
tau = t / T;

trajectories = cell(num_paths,1);   % ✅ COLUMN CELL

%% Extract best trajectory midpoint (control bias)

best_points = best_traj.points;
best_mid = best_traj.control;

%% Generate new trajectories around best

for i = 1:num_paths
    
    % Add small random perturbation around best midpoint
    noise = 0.05 * (rand(1,3) - 0.5);
    P1 = best_mid + noise;
    
    % Quadratic Bezier
    traj = (1 - tau).^2 .* start_pos + ...
           2*(1 - tau).*tau .* P1 + ...
           tau.^2 .* goal_pos;
       
    xd = traj;
    xd_dot = [zeros(1,3); diff(traj)/dt];
    
    trajectories{i}.type = sprintf('adaptive_%d',i);
    trajectories{i}.points = traj;
    trajectories{i}.xd = xd;
    trajectories{i}.xd_dot = xd_dot;
    trajectories{i}.control = P1;
end

end