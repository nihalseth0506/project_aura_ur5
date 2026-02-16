clc;
clear;
close all;

% Initial joint configuration
q = [0, -pi/4, pi/3, -pi/6, pi/4, 0]';

% Target offset (move 5 cm in X)
[~,~,~,~,~,T] = forward_chain_ur5(q');
x_current = T(1:3,4);
x_target = x_current + [0.05; 0; 0];

% Control parameters
K = 2;          % proportional gain
lambda = 0.1;   % damping
dt = 0.01;
steps = 300;

trajectory = zeros(steps,3);

for k = 1:steps
    
    % Current pose
    [~,~,~,~,~,T] = forward_chain_ur5(q');
    x = T(1:3,4);
    
    % Position error
    e = x_target - x;
    
    % Desired Cartesian velocity
    xdot = K * e;
    
    % Jacobian
    J = jacobian_ur5(q');
    
    % DLS inverse
    qdot = J' * inv(J*J' + lambda^2 * eye(6)) * [xdot; 0; 0; 0];
    
    % Integrate
    q = q + qdot * dt;
    
    trajectory(k,:) = x';
    
end

% Plot path
figure;
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3),'LineWidth',2);
hold on;
scatter3(x_target(1), x_target(2), x_target(3), 100, 'r','filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Cartesian Servo using DLS');
grid on;
axis equal;
view(3);
