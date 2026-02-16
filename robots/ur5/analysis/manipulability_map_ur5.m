clc;
clear;
close all;

rng(42);

num_samples = 1000;

positions = zeros(num_samples,3);
manipulability = zeros(num_samples,1);

for i = 1:num_samples
    
    theta_vals = (rand(1,6) - 0.5) * 2*pi;
    
    % Forward chain
    [~,~,~,~,~,T06] = forward_chain_ur5(theta_vals);
    pos = T06(1:3,4);
    positions(i,:) = double(pos)';
    
    % Jacobian
    J = jacobian_ur5(theta_vals);
    
    % Manipulability index
    manipulability(i) = sqrt(det(J*J'));
    
end

% Normalize for color scaling
manipulability = manipulability / max(manipulability);

% 3D Scatter with color mapping
figure;
scatter3(positions(:,1), positions(:,2), positions(:,3), ...
         20, manipulability, 'filled');

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('UR5 Manipulability Distribution');
colorbar;
grid on;
axis equal;
view(3);
