clc;
clear;
close all;

num_samples = 1000;

positions = zeros(num_samples,3);

for i = 1:num_samples
    
    % Random joint angles within reasonable limits
    theta_vals = (rand(1,6) - 0.5) * 2*pi;  
    
    T = forward_kinematics_ur5(theta_vals);
    
    pos = T(1:3,4);
    
    positions(i,:) = double(pos)';
    
end

horizontal_reach = max(sqrt(positions(:,1).^2 + positions(:,2).^2))
max_z = max(positions(:,3))
min_z = min(positions(:,3))


% Plot workspace
figure;
scatter3(positions(:,1), positions(:,2), positions(:,3), 5, 'filled');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('UR5 Reachable Workspace (Sampled)');
grid on;
axis equal;

saveas(gcf, '../../media/images/workspace_sprint1.png');
