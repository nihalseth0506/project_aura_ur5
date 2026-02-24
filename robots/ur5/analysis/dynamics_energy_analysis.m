clc;
clear;
close all;
addpath(genpath(pwd));

% Choose configuration
q_safe = [0; -pi/4; pi/3; -pi/6; pi/4; 0];

res = run_circle_experiment(q_safe, 'adaptive');

q_hist    = res.q_history;
qdot_hist = res.qdot_history;
dt        = res.dt;
steps     = res.steps;

% Compute qddot numerically
qddot_hist = zeros(6,steps);

for k = 2:steps
    qddot_hist(:,k) = (qdot_hist(:,k) - qdot_hist(:,k-1)) / dt;
end

% Compute torques
tau_hist = zeros(6,steps);
power    = zeros(steps,1);

for k = 1:steps
    
    q     = q_hist(:,k);
    qdot  = qdot_hist(:,k);
    qddot = qddot_hist(:,k);
    
    tau = inverse_dynamics_ur5(q, qdot, qddot);
    
    tau_hist(:,k) = tau;   
    % Instantaneous mechanical power
    power(k) = tau' * qdot;
    
end

% Total mechanical energy
energy = sum(power) * dt;

disp("Total Mechanical Energy (Joules):");
disp(energy);

%% =========================================
%  CONSOLIDATED SPRINT 3 DYNAMICS FIGURE
% =========================================

figure('Color','w','Position',[200 100 1000 800]);

tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

%% --- 1. Torque Norm ---
nexttile;
plot(vecnorm(tau_hist),'LineWidth',2);
title('Joint Torque Norm Over Time');
ylabel('||\tau|| (Nm)');
grid on;
box on;
set(gca,'FontSize',12,'LineWidth',1.2)

% Add margin
ax = gca;
x_margin = 0.05 * range(xlim);
y_margin = 0.08 * range(ylim);
xlim([min(xlim)-x_margin, max(xlim)+x_margin])
ylim([min(ylim)-y_margin, max(ylim)+y_margin])

%% --- 2. Mechanical Power ---
nexttile;
plot(power,'LineWidth',2);
title('Mechanical Power Over Time');
ylabel('Power (W)');
grid on;
box on;
set(gca,'FontSize',12,'LineWidth',1.2)

ax = gca;
x_margin = 0.05 * range(xlim);
y_margin = 0.08 * range(ylim);
xlim([min(xlim)-x_margin, max(xlim)+x_margin])
ylim([min(ylim)-y_margin, max(ylim)+y_margin])

%% --- 3. Individual Joint Torques ---
nexttile;
plot(tau_hist','LineWidth',1.4);
title('Individual Joint Torques');
xlabel('Step');
ylabel('Torque (Nm)');
legend('\tau_1','\tau_2','\tau_3','\tau_4','\tau_5','\tau_6','Location','eastoutside');
grid on;
box on;
set(gca,'FontSize',12,'LineWidth',1.2)

ax = gca;
x_margin = 0.05 * range(xlim);
y_margin = 0.08 * range(ylim);
xlim([min(xlim)-x_margin, max(xlim)+x_margin])
ylim([min(ylim)-y_margin, max(ylim)+y_margin])

sgtitle('UR5 Inverse Dynamics Analysis — Circular Motion (r = 0.2 m)','FontSize',14,'FontWeight','bold');

exportgraphics(gcf, ...
'media/images/sprint3/dynamics_analysis_r003_sprint3.png', ...
'Resolution',300);