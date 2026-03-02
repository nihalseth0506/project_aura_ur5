clc;
clear;
close all;

% Define results path
results_path = fullfile('media','images','sprint4');

if ~exist(results_path,'dir')
    mkdir(results_path);
end
%% Run both cases
[E0, tau0, p0] = run_pick_place(0);
[E5, tau5, p5] = run_pick_place(5);

fprintf("Energy without payload: %.3f J\n",E0);
fprintf("Energy with payload:    %.3f J\n",E5);

percentage = (E5 - E0)/E0 * 100;
fprintf("Increase due to payload: %.2f %%\n",percentage);

%% =============================
% Torque Norm Comparison
%% =============================
figure('WindowState','maximized',...
       'MenuBar','none',...
       'ToolBar','none');
plot(vecnorm(tau0),'b','LineWidth',2); hold on;
plot(vecnorm(tau5),'r','LineWidth',2);
title('Torque Norm Comparison');
xlabel('Step');
ylabel('||\tau|| (Nm)');
legend('No Payload','With Payload');
grid on;

%saveas(gcf, fullfile(results_path,'torque_norm_comparison.png'));
exportgraphics(gcf, ...
    fullfile(results_path,'torque_norm_comparison.png'), ...
    'Resolution', 300);   
%% =============================
% Individual Joint Torque
%% =============================
figure('WindowState','maximized',...
       'MenuBar','none',...
       'ToolBar','none');
for j = 1:6
    subplot(3,2,j)
    plot(tau0(j,:),'b'); hold on;
    plot(tau5(j,:),'r');
    title(['Joint ' num2str(j)]);
    legend('No Load','With Load');
    grid on;
end

%saveas(gcf, fullfile(results_path,'joint_torque_comparison.png'));
exportgraphics(gcf, ...
    fullfile(results_path,'joint_torque_comparison.png'), ...
    'Resolution', 300);
%% =============================
% Energy Bar Graph
%% =============================
figure('WindowState','maximized',...
       'MenuBar','none',...
       'ToolBar','none');
bar([E0 E5]);
set(gca,'XTickLabel',{'No Load','With Load'});
ylabel('Total Energy (J)');
title('Energy Consumption Comparison');
grid on;

%saveas(gcf, fullfile(results_path,'energy_comparison.png'));
exportgraphics(gcf, ...
    fullfile(results_path,'energy_comparison.png'), ...
    'Resolution', 300);