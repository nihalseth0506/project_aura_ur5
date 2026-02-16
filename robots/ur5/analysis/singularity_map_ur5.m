clc;
clear;
close all;

rng(42);

num_samples = 1000;

positions = zeros(num_samples,3);
singular_flags = zeros(num_samples,1);

for i = 1:num_samples
    
    theta_vals = (rand(1,6) - 0.5) * 2*pi;
    
    % Forward kinematics
    [~,~,~,~,~,T06] = forward_chain_ur5(theta_vals);
    pos = T06(1:3,4);
    positions(i,:) = double(pos)';
    
    % Jacobian
    J = jacobian_ur5(theta_vals);
    
    if cond(J) > 1000
        singular_flags(i) = 1;
    end
    
end

% Plot
figure;
hold on;

% Non-singular points (blue)
scatter3(positions(singular_flags==0,1), ...
         positions(singular_flags==0,2), ...
         positions(singular_flags==0,3), ...
         8, 'b', 'filled');

% Singular points (red)
scatter3(positions(singular_flags==1,1), ...
         positions(singular_flags==1,2), ...
         positions(singular_flags==1,3), ...
         20, 'r', 'filled');

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('UR5 Workspace with Singularity Mapping');
legend('Non-Singular','Singular');
grid on;
axis equal;
view(3);
