function trajectories = generate_candidate_trajectories_s6(start_pos, goal_pos)

dt = 0.02;
T = 20;
t = (0:dt:T)';
tau = t / T;

trajectories = cell(3,1);

%% Line
line_path = (1 - tau).*start_pos + tau.*goal_pos;

trajectories{1}.type = 'line';
trajectories{1}.points = line_path;
trajectories{1}.xd = line_path;
trajectories{1}.xd_dot = [zeros(1,3); diff(line_path)/dt];

%% Arc
height = 0.1;

arc_path = (1 - tau).*start_pos + tau.*goal_pos;
arc_path(:,3) = arc_path(:,3) + height*sin(pi*tau);

trajectories{2}.type = 'arc';
trajectories{2}.points = arc_path;
trajectories{2}.xd = arc_path;
trajectories{2}.xd_dot = [zeros(1,3); diff(arc_path)/dt];

%% Spline
spline_path = (1 - tau).*start_pos + tau.*goal_pos;
spline_path(:,3) = spline_path(:,3) + 0.05*sin(2*pi*tau);

trajectories{3}.type = 'spline';
trajectories{3}.points = spline_path;
trajectories{3}.xd = spline_path;
trajectories{3}.xd_dot = [zeros(1,3); diff(spline_path)/dt];

end