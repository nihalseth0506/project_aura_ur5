clc;
clear;
close all;
addpath(genpath(pwd));

%% =========================================
%  Base Configuration (SAFE)
% =========================================
q_init = [0; -pi/4; pi/3; -pi/6; pi/4; 0];

% Radius values to test
r_values = [0.03 0.1 0.2 0.3];
n = length(r_values);

% Storage (Fixed)
energy_fixed     = zeros(n,1);
rms_torque_fixed = zeros(n,1);
rms_power_fixed  = zeros(n,1);

% Storage (Adaptive)
energy_adapt     = zeros(n,1);
rms_torque_adapt = zeros(n,1);
rms_power_adapt  = zeros(n,1);

energy_per_meter_fixed     = zeros(n,1);
energy_per_meter_adaptive  = zeros(n,1);

rms_error_fixed     = zeros(n,1);
rms_error_adaptive  = zeros(n,1);

disp("Running radius scaling study (Fixed vs Adaptive)...");

%% =========================================
%  Loop Over Radius Values
% =========================================
for i = 1:n
    
    r_test = r_values(i);

    %% -----------------------------
    % FIXED Damping
    %% -----------------------------
    res_fixed = run_circle_experiment(q_init, 'fixed', r_test);

    [energy_fixed(i), rms_torque_fixed(i), rms_power_fixed(i)] = ...
        compute_energy_metrics(res_fixed);
    % RMS tracking error (Fixed)
    rms_error_fixed(i) = rms(res_fixed.error_history);

    %% -----------------------------
    % ADAPTIVE Damping
    %% -----------------------------
    res_adapt = run_circle_experiment(q_init, 'adaptive', r_test);

    [energy_adapt(i), rms_torque_adapt(i), rms_power_adapt(i)] = ...
        compute_energy_metrics(res_adapt);
    
    % RMS tracking error (Adaptive)
    rms_error_adaptive(i) = rms(res_adapt.error_history);

    %%
    path_length = 2*pi*r_test;

    energy_per_meter_fixed(i)    = energy_fixed(i)    / path_length;
    energy_per_meter_adaptive(i) = energy_adapt(i) / path_length;

end

%% =========================================
%  Plot Energy vs Radius
% =========================================
figure('Position',[100 100 900 700]);

plot(r_values, energy_fixed,'-o','LineWidth',2); hold on;
plot(r_values, energy_adapt,'-o','LineWidth',2);

xlabel('Circle Radius (m)');
ylabel('Total Mechanical Energy (J)');
title('Energy vs Radius — Fixed vs Adaptive');
legend('Fixed Damping','Adaptive Damping','Location','northeast');

grid on;
box on;

ax = gca;
ax.FontSize = 12;
ax.LineWidth = 1.2;
ax.Units = 'normalized';
ax.Position = [0.18 0.18 0.72 0.72];
ax.ActivePositionProperty = 'position';

set(gcf,'Color','w');

exportgraphics(gcf, ...
    fullfile('media','images','sprint3','energy_vs_radius_sprint3.png'), ...
    'Resolution',300, ...
    'BackgroundColor','white');
%% =========================================
%  Plot RMS Torque vs Radius
% =========================================
figure('Position',[100 100 900 700]);

plot(r_values, rms_torque_fixed,'-o','LineWidth',2); hold on;
plot(r_values, rms_torque_adapt,'-o','LineWidth',2);

xlabel('Circle Radius (m)');
ylabel('RMS Joint Torque Norm (Nm)');
title('Torque Demand vs Radius');
legend('Fixed Damping','Adaptive Damping','Location','northwest');

grid on;
box on;

ax = gca;
ax.FontSize = 12;
ax.LineWidth = 1.2;
ax.Units = 'normalized';
ax.Position = [0.18 0.18 0.72 0.72];
ax.ActivePositionProperty = 'position';

set(gcf,'Color','w');

exportgraphics(gcf, ...
    fullfile('media','images','sprint3','torque_vs_radius_sprint3.png'), ...
    'Resolution',300, ...
    'BackgroundColor','white');
%% =========================================
%  Plot RMS Power vs Radius
% =========================================
figure;
plot(r_values, rms_power_fixed,'-o','LineWidth',2); hold on;
plot(r_values, rms_power_adapt,'-o','LineWidth',2);
xlabel('Circle Radius (m)');
ylabel('RMS Mechanical Power (W)');
title('Power Consumption vs Radius');
legend('Fixed Damping','Adaptive Damping');
grid on;

disp("Radius scaling study complete.");

%% ========================
%  Plot Energy Per Meter
% ========================

figure;
plot(r_values, energy_per_meter_fixed,'-o','LineWidth',2); hold on;
plot(r_values, energy_per_meter_adaptive,'-o','LineWidth',2);

xlabel('Circle Radius (m)');
ylabel('Energy per Meter (J/m)');
title('Energy Efficiency vs Radius — Fixed vs Adaptive');
legend('Fixed Damping','Adaptive Damping','Location','best');
grid on;

%% ========================
%  Tracking Error vs Radius
% ========================

figure('Position',[100 100 900 700]);   % Larger figure canvas

plot(r_values, rms_error_fixed,'-o','LineWidth',2); hold on;
plot(r_values, rms_error_adaptive,'-o','LineWidth',2);

xlabel('Circle Radius (m)');
ylabel('RMS Tracking Error (m)');
title('Tracking Accuracy vs Radius — Fixed vs Adaptive');
legend('Fixed Damping','Adaptive Damping','Location','northwest');
grid on;
box on;

ax = gca;
ax.FontSize = 12;
ax.LineWidth = 1.2;

% Force MATLAB to respect manual positioning
ax.Units = 'normalized';
ax.Position = [0.18 0.18 0.72 0.72];
ax.ActivePositionProperty = 'position';

set(gcf,'Color','w');

exportgraphics(gcf, ...
    fullfile('media','images','sprint3','tracking_error_vs_radius_sprint3.png'), ...
    'Resolution',300, ...
    'BackgroundColor','white');