clc
clear
close all

addpath(genpath(pwd))
rng(1);
model = 'ur5_controller';
load_system(model)

%% Start and goal positions

start_pos = [0.4 0.1 0.3];
goal_pos  = [0.5 -0.2 0.15];

%% Mode selection
mode = 'random';   % 'fixed' or 'random'

%% Run optimization

[best_energy_traj, best_cost_traj, energy_values, cost_values, trajectories] = ...
    optimize_trajectory_energy_s6(start_pos,goal_pos,mode);

colors = lines(length(trajectories));

%% =========================
% FIGURE
%% =========================
figure

%% =========================
% SUBPLOT 1 → Energy-based
%% =========================
subplot(1,2,1)
hold on
grid on
axis equal
title("Energy Optimal Path")

for i = 1:length(trajectories)
    
    traj = trajectories{i}.points;
    
    if strcmp(mode,'fixed')
        % 🎨 Fixed → colored lines
        colors_fixed = lines(length(trajectories));
        plot3(traj(:,1),traj(:,2),traj(:,3),...
            'Color',colors_fixed(i,:),'LineWidth',2)
        
    else
        % 🎯 Random mode (your current logic)
        if contains(trajectories{i}.type,'adaptive')
            plot3(traj(:,1),traj(:,2),traj(:,3),...
                ':','Color',[0 0.4 1],'LineWidth',2)
        elseif isinf(cost_values(i))
            plot3(traj(:,1),traj(:,2),traj(:,3),...
                '--','Color',[0.6 0.6 0.6],'LineWidth',1)
        else
            plot3(traj(:,1),traj(:,2),traj(:,3),...
                'Color',[0.2 0.2 0.2],'LineWidth',1.2)
        end
    end
    
end

% Best energy trajectory
best_energy = best_energy_traj.points;

plot3(best_energy(:,1),best_energy(:,2),best_energy(:,3),...
    'k--','LineWidth',3)

% Start & Goal
plot3(start_pos(1),start_pos(2),start_pos(3),'ko','MarkerFaceColor','k')
plot3(goal_pos(1),goal_pos(2),goal_pos(3),'ks','MarkerFaceColor','k')

xlabel("X"); ylabel("Y"); zlabel("Z")
view(3)
view(-55.5495, 9.9052)


%% =========================
% SUBPLOT 2 → Cost-based
%% =========================
subplot(1,2,2)
hold on
grid on
axis equal
title("Energy + Manip Optimal Path")

for i = 1:length(trajectories)
    
    traj = trajectories{i}.points;
    
    if strcmp(mode,'fixed')
        % 🎨 Fixed → colored lines
        colors_fixed = lines(length(trajectories));
        plot3(traj(:,1),traj(:,2),traj(:,3),...
            'Color',colors_fixed(i,:),'LineWidth',2)
        
    else
        % 🎯 Random mode (your current logic)
        if contains(trajectories{i}.type,'adaptive')
            plot3(traj(:,1),traj(:,2),traj(:,3),...
                ':','Color',[0 0.4 1],'LineWidth',2)
        elseif isinf(cost_values(i))
            plot3(traj(:,1),traj(:,2),traj(:,3),...
                '--','Color',[0.6 0.6 0.6],'LineWidth',1)
        else
            plot3(traj(:,1),traj(:,2),traj(:,3),...
                'Color',[0.2 0.2 0.2],'LineWidth',1.2)
        end
    end
    
end

% Best cost trajectory
best_cost = best_cost_traj.points;

% =========================
% EXPORT BEST TRAJECTORY FOR ROS2
% =========================

% % Use best cost trajectory
xd = best_cost_traj.points;

% Ensure same start
xd(1,:) = start_pos;

% Time step (must match ROS2)
dt = 0.02;

% Velocity
xd_dot = [zeros(1,3); diff(xd)/dt];

% Smooth velocity
xd_dot = smoothdata(xd_dot, 'movmean', 5);

% Limit velocity
v_max = 0.5;
xd_dot = max(min(xd_dot, v_max), -v_max);

% Combine
data = [xd, xd_dot];

% 🔥 SAVE TWO FILES

% (1) For ROS2
writematrix(data, ...
    'D:/projects/project_aura_ur5/robots/data_logs/trajectory.csv');

% (2) For comparison (PLANNED ONLY)
writematrix(xd, ...
    'D:/projects/project_aura_ur5/robots/data_logs/planned_traj_s7.csv');

fprintf("Trajectory exported for ROS2 + comparison\n");

%%
plot3(best_cost(:,1),best_cost(:,2),best_cost(:,3),...
    'k--','LineWidth',3)

% Start & Goal
plot3(start_pos(1),start_pos(2),start_pos(3),'ko','MarkerFaceColor','k')
plot3(goal_pos(1),goal_pos(2),goal_pos(3),'ks','MarkerFaceColor','k')

xlabel("X"); ylabel("Y"); zlabel("Z")
view(3)
view(-55.5495, 9.9052)

%% =========================
% GLOBAL LEGEND (CLEAN)
%% =========================

if strcmp(mode,'fixed')
    
    legend_entries = cell(1,length(trajectories)+1);
    
    for i = 1:length(trajectories)
        legend_entries{i} = trajectories{i}.type;
    end
    
    legend_entries{end} = 'Optimal';
    
    legend(legend_entries,'Position',[0.4 0.9 0.2 0.05])
    
else
    % Random mode legend (your current one)
    h1 = plot3(nan,nan,nan,'-','Color',[0.2 0.2 0.2],'LineWidth',1.5);
    h2 = plot3(nan,nan,nan,':','Color',[0 0.4 1],'LineWidth',2);
    h3 = plot3(nan,nan,nan,'--','Color',[0.6 0.6 0.6],'LineWidth',1.5);
    h4 = plot3(nan,nan,nan,'k--','LineWidth',2);

    legend([h1 h2 h3 h4], ...
        {'Random Valid','Adaptive','Rejected','Optimal'}, ...
        'Position',[0.4 0.9 0.2 0.05])
end

%% =========================
% RESULT TEXT BOX (CENTERED)
%% =========================

result_text = sprintf('Results:\n');

for i = 1:length(trajectories)
    
    if isinf(cost_values(i))
        line = sprintf('\n%s -> REJECTED\n', trajectories{i}.type);
    else
        line = sprintf('\n%s -> E= %.1f \n', ...
            trajectories{i}.type, energy_values(i));
    end
    
    result_text = strcat(result_text, line);
end

result_text = strcat(result_text, ...
    sprintf('\nEnergy Best: %s\nCost Best: %s', ...
    best_energy_traj.type, best_cost_traj.type));

annotation('textbox',[0.4625 0.15 0.3 0.725], ...
    'String', result_text, ...
    'FitBoxToText','on', ...
    'BackgroundColor','white', ...
    'EdgeColor','black', ...
    'FontSize',10, 'Interpreter','none','Margin', 2);

%% =========================
% SAVE FIGURE (HIGH QUALITY)
%% =========================

% % Create folder if it doesn't exist
% save_folder = fullfile(pwd,'media','images','sprint6');
% 
% if ~exist(save_folder, 'dir')
%     mkdir(save_folder);
% end
% 
% % Set figure to full screen
% set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
% set(gcf,'Color','w');  % white background
% % File name based on mode
% filename = fullfile(save_folder, ['sprint6_' mode '.png']);
% 
% % Save high resolution image
% exportgraphics(gcf, filename, 'Resolution', 300);
% 
% fprintf("Figure saved at: %s\n", filename);