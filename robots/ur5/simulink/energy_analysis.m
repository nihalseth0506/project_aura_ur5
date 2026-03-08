clc
clear
close all

model = 'ur5_controller';
paramBlock = [model '/Constant'];

% Define results path
results_path = fullfile('media','images','sprint5');

if ~exist(results_path,'dir')
    mkdir(results_path);
end

%% Circle

set_param(paramBlock,'Value','1')
simOut = sim(model);

E_circle = squeeze(simOut.energy_log);
E_circle_total = E_circle(end);

W_circle = squeeze(simOut.manipulability_log);

%% Square

set_param(paramBlock,'Value','2')
simOut = sim(model);

E_square = squeeze(simOut.energy_log);
E_square_total = E_square(end);

W_square = squeeze(simOut.manipulability_log);

%% Figure8

set_param(paramBlock,'Value','3')
simOut = sim(model);

E_fig8 = squeeze(simOut.energy_log);
E_fig8_total = E_fig8(end);

W_fig8 = squeeze(simOut.manipulability_log);

%% Spiral

set_param(paramBlock,'Value','4')
simOut = sim(model);

E_spiral = squeeze(simOut.energy_log);
E_spiral_total = E_spiral(end);

W_spiral = squeeze(simOut.manipulability_log);


%% ENERGY COMPARISON BAR CHART
figure('WindowState','maximized',...
       'MenuBar','none',...
       'ToolBar','none');

energy = [E_circle_total E_square_total E_fig8_total E_spiral_total];

labels = {'Circle','Square','Figure8','Spiral'};

bar(energy)

set(gca,'XTickLabel',labels)

ylabel('Total Energy (J)')
title('Energy Consumption Comparison')

grid on

% Save figure
exportgraphics(gcf, ...
    fullfile(results_path,'energy_comparison_bar_chart.png'), ...
    'Resolution',300);
set(gcf,'Color','white')

%% MANIPULABILITY VS TIME
figure('WindowState','maximized',...
       'MenuBar','none',...
       'ToolBar','none');

plot(W_circle,'LineWidth',1.5)
hold on
plot(W_square,'LineWidth',1.5)
plot(W_fig8,'LineWidth',1.5)
plot(W_spiral,'LineWidth',1.5)

legend('Circle','Square','Figure8','Spiral')
axis tight

ylabel('Manipulability')
xlabel('Time Step')

title('Manipulability During Trajectories')

grid on

% Save figure
exportgraphics(gcf, ...
    fullfile(results_path,'manipulability_vs_time.png'), ...
    'Resolution',300);

set(gcf,'Color','white')
%% ENERGY VS TIME
figure('WindowState','maximized',...
       'MenuBar','none',...
       'ToolBar','none');

plot(E_circle,'LineWidth',2)
hold on
plot(E_square,'LineWidth',2)
plot(E_fig8,'LineWidth',2)
plot(E_spiral,'LineWidth',2)

legend('Circle','Square','Figure8','Spiral')
axis tight

xlabel('Time Step')
ylabel('Energy (J)')

title('Energy Consumption Over Time')

grid on

% Save figure
exportgraphics(gcf, ...
    fullfile(results_path,'energy_vs_time.png'), ...
    'Resolution',300);
set(gcf,'Color','white')