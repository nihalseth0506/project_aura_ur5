clc;
clear;
close all;
addpath(genpath(pwd));

% Safe configuration
q_safe = [0, -pi/4, pi/3, -pi/6, pi/4, 0]';

% Singular configuration
q_sing = [0 0 0 0 0 0]';

disp("Running experiments...");

% Run all cases
res_safe_fixed      = run_circle_experiment(q_safe, 'fixed');
res_sing_fixed      = run_circle_experiment(q_sing, 'fixed');
res_safe_adaptive   = run_circle_experiment(q_safe, 'adaptive');
res_sing_adaptive   = run_circle_experiment(q_sing, 'adaptive');

%% ==============================
%  FIGURE 1 — Safe Config Comparison
% ===============================

figure;
subplot(2,1,1);
plot(res_safe_fixed.error_history,'LineWidth',1.5); hold on;
plot(res_safe_adaptive.error_history,'LineWidth',1.5);
title('SAFE Configuration — Position Error');
legend('Fixed Damping (\lambda = 0.1)', ...
       'Adaptive Damping (Manipulability-Based)');
grid on;

subplot(2,1,2);
plot(res_safe_fixed.qdot_norm,'LineWidth',1.5); hold on;
plot(res_safe_adaptive.qdot_norm,'LineWidth',1.5);
title('SAFE Configuration — Joint Velocity Norm');
legend('Fixed Damping (\lambda = 0.1)', ...
       'Adaptive Damping (Manipulability-Based)');
grid on;

%% ==============================
%  FIGURE 2 — Singular Config Comparison
% ===============================

figure;
subplot(2,1,1);
plot(res_sing_fixed.error_history,'LineWidth',1.5); hold on;
plot(res_sing_adaptive.error_history,'LineWidth',1.5);
title('SINGULAR Configuration — Position Error');
legend('Fixed Damping (\lambda = 0.1)', ...
       'Adaptive Damping (Manipulability-Based)');
grid on;

subplot(2,1,2);
plot(res_sing_fixed.qdot_norm,'LineWidth',1.5); hold on;
plot(res_sing_adaptive.qdot_norm,'LineWidth',1.5);
title('SINGULAR Configuration — Joint Velocity Norm');
legend('Fixed Damping (\lambda = 0.1)', ...
       'Adaptive Damping (Manipulability-Based)');
grid on;

%% ==============================
%  FIGURE 3 — Manipulability Comparison
% ===============================

figure;

subplot(2,1,1);
plot(res_safe_fixed.manip_history,'LineWidth',2);
title('SAFE Configuration — Manipulability');
xlabel('Step');
ylabel('w');
grid on;

subplot(2,1,2);
plot(res_sing_fixed.manip_history,'LineWidth',2);
title('SINGULAR Configuration — Manipulability');
xlabel('Step');
ylabel('w');
grid on;

sgtitle('Manipulability Index Over Time');

%% =========================================
%  FIGURE — 4 Circle Tracking Comparisons
% ==========================================

figure;

%% Extract parameters from SAFE case
dt    = res_safe_fixed.dt;
steps = res_safe_fixed.steps;
r     = res_safe_fixed.r;
omega = res_safe_fixed.omega;

%% =============================
% Desired trajectories
% ==============================

t_vec = (1:steps)' * dt;

% SAFE desired
x0_safe = res_safe_fixed.x0;
xd_safe = zeros(steps,3);
for k = 1:steps
    t = t_vec(k);
    xd_safe(k,:) = [ x0_safe(1) + r*cos(omega*t), ...
                     x0_safe(2) + r*sin(omega*t), ...
                     x0_safe(3) ];
end

% SINGULAR desired
x0_sing = res_sing_fixed.x0;
xd_sing = zeros(steps,3);
for k = 1:steps
    t = t_vec(k);
    xd_sing(k,:) = [ x0_sing(1) + r*cos(omega*t), ...
                     x0_sing(2) + r*sin(omega*t), ...
                     x0_sing(3) ];
end

%% =============================
% SAFE — FIXED
% ==============================
subplot(2,2,1);

plot3(xd_safe(:,1), xd_safe(:,2), xd_safe(:,3),'k--','LineWidth',1.5); hold on;
plot3(res_safe_fixed.trajectory(:,1), ...
      res_safe_fixed.trajectory(:,2), ...
      res_safe_fixed.trajectory(:,3),'b','LineWidth',2);

title('SAFE — Fixed Damping');
grid on; axis equal; view(3);

% ---- Store SAFE axis limits for later reuse ----
safe_xlim = xlim;
safe_ylim = ylim;
safe_zlim = zlim;

% ---- Center Arrow (SAFE FIXED) ----
traj = res_safe_fixed.trajectory;
center_point = mean(traj);
[~, arrow_idx] = min(vecnorm(traj - center_point, 2, 2));

p1 = traj(arrow_idx,:);
p2 = traj(arrow_idx+1,:);
dir_vec = (p2 - p1) / norm(p2 - p1);

quiver3(p1(1), p1(2), p1(3), ...
        0.02*dir_vec(1), ...
        0.02*dir_vec(2), ...
        0.02*dir_vec(3), ...
        0,'k','LineWidth',2,'MaxHeadSize',2);

%% =============================
% SAFE — ADAPTIVE
% ==============================
subplot(2,2,2);

plot3(xd_safe(:,1), xd_safe(:,2), xd_safe(:,3),'k--','LineWidth',1.5); hold on;
plot3(res_safe_adaptive.trajectory(:,1), ...
      res_safe_adaptive.trajectory(:,2), ...
      res_safe_adaptive.trajectory(:,3),'r','LineWidth',2);

title('SAFE — Adaptive Damping');
grid on; axis equal; view(3);

xlim(safe_xlim); ylim(safe_ylim); zlim(safe_zlim);

% ---- Center Arrow (SAFE ADAPTIVE) ----
traj = res_safe_adaptive.trajectory;
center_point = mean(traj);
[~, arrow_idx] = min(vecnorm(traj - center_point, 2, 2));

p1 = traj(arrow_idx,:);
p2 = traj(arrow_idx+1,:);
dir_vec = (p2 - p1) / norm(p2 - p1);

quiver3(p1(1), p1(2), p1(3), ...
        0.02*dir_vec(1), ...
        0.02*dir_vec(2), ...
        0.02*dir_vec(3), ...
        0,'k','LineWidth',2,'MaxHeadSize',2);

%% =============================
% SINGULAR — FIXED
% ==============================
subplot(2,2,3);

plot3(xd_sing(:,1), xd_sing(:,2), xd_sing(:,3),'k--','LineWidth',1.5); hold on;
plot3(res_sing_fixed.trajectory(:,1), ...
      res_sing_fixed.trajectory(:,2), ...
      res_sing_fixed.trajectory(:,3),'b','LineWidth',2);

title('SINGULAR — Fixed Damping');
grid on; axis equal; view(3);

% ---- Center Arrow (SINGULAR FIXED) ----
traj = res_sing_fixed.trajectory;
center_point = mean(traj);
[~, arrow_idx] = min(vecnorm(traj - center_point, 2, 2));

p1 = traj(arrow_idx,:);
p2 = traj(arrow_idx+1,:);
dir_vec = (p2 - p1) / norm(p2 - p1);

quiver3(p1(1), p1(2), p1(3), ...
        0.02*dir_vec(1), ...
        0.02*dir_vec(2), ...
        0.02*dir_vec(3), ...
        0,'k','LineWidth',2,'MaxHeadSize',2);

%% =============================
% SINGULAR — ADAPTIVE
% ==============================
subplot(2,2,4);

plot3(xd_sing(:,1), xd_sing(:,2), xd_sing(:,3),'k--','LineWidth',1.5); hold on;
plot3(res_sing_adaptive.trajectory(:,1), ...
      res_sing_adaptive.trajectory(:,2), ...
      res_sing_adaptive.trajectory(:,3),'r','LineWidth',2);

title('SINGULAR — Adaptive Damping');
grid on; axis equal; view(3);

% ---- Center Arrow (SINGULAR ADAPTIVE) ----
traj = res_sing_adaptive.trajectory;
center_point = mean(traj);
[~, arrow_idx] = min(vecnorm(traj - center_point, 2, 2));

p1 = traj(arrow_idx,:);
p2 = traj(arrow_idx+1,:);
dir_vec = (p2 - p1) / norm(p2 - p1);

quiver3(p1(1), p1(2), p1(3), ...
        0.02*dir_vec(1), ...
        0.02*dir_vec(2), ...
        0.02*dir_vec(3), ...
        0,'k','LineWidth',2,'MaxHeadSize',2);

sgtitle('UR5 Circular Cartesian Tracking — Safe vs Singular Configurations');


%% =========================================
%  SAVE ALL SPRINT 2 FIGURES
% ==========================================

figs = findall(0,'Type','figure');

% Make sure figures are ordered properly
% (they were created in order 1 → 4)

% 1️⃣ Safe Comparison
exportgraphics(figs(4), ...
    'media/images/sprint2/safe_config_comparison_sprint2.png', ...
    'Resolution',300);

% 2️⃣ Singular Comparison
exportgraphics(figs(3), ...
    'media/images/sprint2/singular_config_comparison_sprint2.png', ...
    'Resolution',300);

% 3️⃣ Manipulability Plot
exportgraphics(figs(2), ...
    'media/images/sprint2/manipulability_index_sprint2.png', ...
    'Resolution',300);

% 4️⃣ Circle Tracking 4-Panel Comparison
exportgraphics(figs(1), ...
    'media/images/sprint2/circle_tracking_comparison_sprint2.png', ...
    'Resolution',300);

disp("Sprint 2 figures saved successfully.");
