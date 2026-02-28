function animate_ur5_stick(q_traj, waypoints)

figure;
axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(135,25);
hold on;

xlim([-1 1]);
ylim([-1 1]);
zlim([0 1]);

% Extract pick and place markers
x_pick  = waypoints(:,2);
x_place = waypoints(:,5);

% Plot static markers
plot3(x_pick(1),  x_pick(2),  x_pick(3),  'go','MarkerSize',10,'LineWidth',2);
plot3(x_place(1), x_place(2), x_place(3), 'ro','MarkerSize',10,'LineWidth',2);

ee_path = [];

for k = 1:size(q_traj,2)
    
    q = q_traj(:,k);
    
    [T01,T02,T03,T04,T05,T06] = forward_chain_ur5(q');
    
    p0 = [0;0;0];
    p1 = T01(1:3,4);
    p2 = T02(1:3,4);
    p3 = T03(1:3,4);
    p4 = T04(1:3,4);
    p5 = T05(1:3,4);
    p6 = T06(1:3,4);   % end-effector
    
    P = [p0 p1 p2 p3 p4 p5 p6];
    
    ee_path = [ee_path p6];
    
    cla;
    
    % Robot links
    plot3(P(1,:), P(2,:), P(3,:), '-o','LineWidth',2);
    hold on;
    
    % End-effector path trace
    plot3(ee_path(1,:), ee_path(2,:), ee_path(3,:), '--k','LineWidth',1.5);
    
    % Re-plot markers (since cla clears them)
    plot3(x_pick(1),  x_pick(2),  x_pick(3),  'go','MarkerSize',10,'LineWidth',2);
    plot3(x_place(1), x_place(2), x_place(3), 'ro','MarkerSize',10,'LineWidth',2);
    
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([0 1]);
    
    drawnow;
end

legend('Robot','Path','Pick','Place');

end