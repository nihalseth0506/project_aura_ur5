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

% Plot torque norm
figure;
plot(vecnorm(tau_hist),'LineWidth',2);
title('Joint Torque Norm Over Time');
xlabel('Step');
ylabel('||tau||');
grid on;

% Plot power
figure;
plot(power,'LineWidth',2);
title('Mechanical Power Over Time');
xlabel('Step');
ylabel('Power (W)');
grid on;

figure;
plot(tau_hist','LineWidth',1.5);
title('Individual Joint Torques');
xlabel('Step');
ylabel('Torque (Nm)');
legend('\tau_1','\tau_2','\tau_3','\tau_4','\tau_5','\tau_6');
grid on;