clc; clear; close all;

addpath(genpath(pwd))

%% =========================
% LOAD DATA
%% =========================

% Planned (MATLAB optimal)
planned_xyz = readmatrix('planned_traj_s7.csv');

% Executed (ROS2)
executed = readmatrix('ros_executed.csv');

% Match sizes
N = min(size(planned_xyz,1), size(executed,1));
planned_xyz = planned_xyz(1:N,:);
executed     = executed(1:N,:);

%% =========================
% EXTRACT START & GOAL
%% =========================

start_pos = planned_xyz(1,:);
goal_pos  = planned_xyz(end,:);

%% =========================
% 3D TRAJECTORY PLOT
%% =========================

figure
plot3(planned_xyz(:,1), planned_xyz(:,2), planned_xyz(:,3), ...
    'k--','LineWidth',2)
hold on

plot3(executed(:,1), executed(:,2), executed(:,3), ...
    'r','LineWidth',2)

% Start & Goal markers
plot3(start_pos(1), start_pos(2), start_pos(3), ...
    'go','MarkerSize',8,'MarkerFaceColor','g')

plot3(goal_pos(1), goal_pos(2), goal_pos(3), ...
    'ro','MarkerSize',8,'MarkerFaceColor','r')

grid on
axis equal
xlabel('X'); ylabel('Y'); zlabel('Z')

legend('Planned (MATLAB)','Executed (ROS2)','Start','Goal','Location','best')

title('MATLAB vs ROS2 Trajectory Tracking')
%view(3)
view(-55.6912,11.8261)
%% =========================
% TRACKING ERROR
%% =========================

error = vecnorm(planned_xyz - executed, 2, 2);

figure(2)
plot(error,'LineWidth',2)
grid on

title('Tracking Error')
xlabel('Time Step')
ylabel('Error (m)')

% =========================
% 🔥 TEXTBOX (Mean & Max)
% =========================
mean_err = mean(error);
max_err  = max(error);

txt = sprintf('Mean Error: %.4f m\nMax Error: %.4f m', ...
    mean_err, max_err);

annotation('textbox',[0.80 0.75 0.25 0.1], ...
    'String', txt, ...
    'FitBoxToText','on', ...
    'BackgroundColor','white', ...
    'EdgeColor','black', ...
    'FontSize',10);

%% =========================
% SAVE FIGURES (FINAL)
%% =========================

save_folder = fullfile(pwd,'media','images','sprint7');

% Create folder if not exists
if ~exist(save_folder, 'dir')
    mkdir(save_folder);
end

%% -------- SAVE TRAJECTORY FIGURE --------
figure(1)
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf,'Color','w');

exportgraphics(gcf, fullfile(save_folder, 'trajectory_comparison.png'), ...
    'Resolution', 300);

%% -------- SAVE ERROR FIGURE --------
figure(2)
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf,'Color','w');

exportgraphics(gcf, fullfile(save_folder, 'tracking_error.png'), ...
    'Resolution', 300);