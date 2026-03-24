function [best_energy_traj, best_cost_traj, energy_values, cost_values, trajectories] = ...
    optimize_trajectory_energy_s6(start_pos,goal_pos,mode)

%% =========================
% Generate trajectories
%% =========================
if strcmp(mode,'fixed')
    trajectories = generate_candidate_trajectories_s6(start_pos,goal_pos);
else
    num_paths = 10;
    trajectories = generate_random_trajectories_s6(start_pos,goal_pos,num_paths);
end

num_traj = length(trajectories);

%% =========================
% Initialize
%% =========================
energy_values = inf(num_traj,1);   % default = rejected
manip_values  = zeros(num_traj,1);
cost_values   = inf(num_traj,1);

lambda = 10;

% Constraint thresholds
manip_threshold = 0.0015;
tau_limit = 150;   % adjust if needed

model = 'ur5_controller';

%% =========================
% Loop over trajectories
%% =========================
for i = 1:length(trajectories)
    
    try
        %% Time setup
        N = size(trajectories{i}.xd,1);
        dt = 0.02;
        T = 20;
        t = (0:dt:T)';
        
        %% External signals
        xd_external = timeseries(double(trajectories{i}.xd), t);
        xd_dot_external = timeseries(double(trajectories{i}.xd_dot), t);
        
        assignin('base','xd_external',xd_external);
        assignin('base','xd_dot_external',xd_dot_external);
        
        %% Run simulation
        simOut = sim(model);
        
        energy = squeeze(simOut.energy_log);
        manip  = squeeze(simOut.manipulability_log);
        
        % OPTIONAL (only if available in your model)
        if isfield(simOut,'tau_log')
            tau = squeeze(simOut.tau_log);
            max_tau = max(abs(tau(:)));
        else
            max_tau = 0; % skip torque constraint if not available
        end
        
        %% =========================
        % Constraint Filtering
        %% =========================
        if min(manip) < manip_threshold || max_tau > tau_limit
            fprintf("%s : REJECTED (constraint violation)\n", trajectories{i}.type);
            continue;   % skip this trajectory
        end
        
        %% =========================
        % Cost computation
        %% =========================
        
        % Power approximation
        power = diff(energy) / dt;
        power = [power(1); power];
        
        % Avoid division by zero
        manip_safe = max(manip, 1e-6);
        
        penalty = lambda * (1 ./ manip_safe);
        
        instant_cost = abs(power) + penalty;
        
        cost_integrated = sum(instant_cost) * dt;
        
        %% Store values
        energy_values(i) = energy(end);
        manip_values(i)  = min(manip);
        cost_values(i)   = cost_integrated;
        
    catch
        fprintf("%s : ERROR (simulation failed)\n", trajectories{i}.type);
        continue;
    end
    
end

%% =========================
% Select best trajectories
%% =========================
[~,idx_cost] = min(cost_values);
[~,idx_energy] = min(energy_values);

best_energy_traj = trajectories{idx_energy};
best_cost_traj   = trajectories{idx_cost};

%% =========================
% ADAPTIVE SAMPLING (SECOND PASS)
%% =========================

if ~strcmp(mode,'fixed')   % only for random mode
    
    num_adaptive = 5;
    
    adaptive_trajs = generate_adaptive_trajectories_s6( ...
        start_pos,goal_pos, ...
        best_cost_traj, ...
        num_adaptive);
    
    % Append to existing trajectories
    trajectories = [trajectories; adaptive_trajs];
    
    % Update size
    num_traj_new = length(trajectories);
    
    % Extend arrays
    energy_values(num_traj_new) = inf;
    cost_values(num_traj_new)   = inf;
    manip_values(num_traj_new)  = 0;
    
    %% Evaluate ONLY new trajectories
    for i = (num_traj+1):num_traj_new
        
        try
            
            N = size(trajectories{i}.xd,1);
            dt = 0.02;
            T = 20;
            t = (0:dt:T)';
            
            xd_external = timeseries(double(trajectories{i}.xd), t);
            xd_dot_external = timeseries(double(trajectories{i}.xd_dot), t);
            
            assignin('base','xd_external',xd_external);
            assignin('base','xd_dot_external',xd_dot_external);
            
            simOut = sim(model);
            
            energy = squeeze(simOut.energy_log);
            manip  = squeeze(simOut.manipulability_log);
            
            if isfield(simOut,'tau_log')
                tau = squeeze(simOut.tau_log);
                max_tau = max(abs(tau(:)));
            else
                max_tau = 0;
            end
            
            if min(manip) < manip_threshold || max_tau > tau_limit
                fprintf("%s : REJECTED (adaptive)\n", trajectories{i}.type);
                continue;
            end
            
            power = diff(energy)/dt;
            power = [power(1); power];
            
            manip_safe = max(manip,1e-6);
            penalty = lambda*(1./manip_safe);
            
            instant_cost = abs(power) + penalty;
            cost_integrated = sum(instant_cost)*dt;
            
            energy_values(i) = energy(end);
            manip_values(i)  = min(manip);
            cost_values(i)   = cost_integrated;
            
        catch
            fprintf("%s : ERROR (adaptive)\n", trajectories{i}.type);
        end
        
    end
    
    % FINAL BEST after adaptive
    [~,idx_cost] = min(cost_values);
    [~,idx_energy] = min(energy_values);
    
    best_energy_traj = trajectories{idx_energy};
    best_cost_traj   = trajectories{idx_cost};
    
end
%% =========================
% Print results
%% =========================
fprintf("\nResults:\n")

for i=1:length(trajectories)
    
    if isinf(cost_values(i))
        fprintf("%s : REJECTED\n", trajectories{i}.type);
    else
        fprintf("%s : Energy=%.2f | Manip=%.4f | Cost=%.2f\n", ...
            trajectories{i}.type, ...
            energy_values(i), ...
            manip_values(i), ...
            cost_values(i));
    end
    
end

fprintf("\nEnergy-only best: %s\n", trajectories{idx_energy}.type);
fprintf("Energy+Manip best: %s\n", trajectories{idx_cost}.type);

end