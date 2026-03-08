function animate_ur5_simulink(q_traj, xd_traj)

figure('Units','pixels',...
       'Position',[100 100 1400 900],...
       'Color','white',...
       'MenuBar','none',...
       'ToolBar','none');

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

title('UR5 Cartesian Trajectory Tracking','FontSize',16)

xlim([-1 1])
ylim([-1 1])
zlim([0 1])

view(210,30)
hold on

%% =============================
% VIDEO SETUP (STABLE VERSION)
%% =============================
results_path = fullfile('media','images','sprint5');
if ~exist(results_path,'dir')
    mkdir(results_path);
end

video_file = fullfile(results_path,'sprint5_animation_spiral.avi');

vidObj = VideoWriter(video_file,'Motion JPEG AVI');
vidObj.FrameRate = 80;
vidObj.Quality = 100;
open(vidObj);

%% Robot links
robot_handle = plot3(0,0,0,'-o',...
    'LineWidth',4,...
    'MarkerSize',6,...
    'MarkerFaceColor',[0 0.4 0.8]);

%% Actual path
actual_handle = plot3(0,0,0,'r','LineWidth',2);

%% Desired path
desired_handle = plot3(xd_traj(1,:),xd_traj(2,:),xd_traj(3,:),...
    'g--','LineWidth',2);

legend({'Robot','Actual EE Path','Desired Path'})

ee_path = [];

% Joint label handles
joint_labels = gobjects(7,1);   % J1..J6 + EE

for k = 1:size(q_traj,2)
    
    camorbit(0.1,0,'data',[0 0 1])

    q = q_traj(:,k);

    [T01,T02,T03,T04,T05,T06] = forward_chain_ur5(q');

    p1 = T01(1:3,4);
    p2 = T02(1:3,4);
    p3 = T03(1:3,4);
    p4 = T04(1:3,4);
    p5 = T05(1:3,4);
    p6 = T06(1:3,4);

    %% joint positions
    J1 = [0;0;0];
    J2 = p1;
    J3 = p2;
    J4 = p3;
    J5 = p4;
    J6 = p5;
    EE = p6;
    
    joint_positions = [J1 J2 J3 J4 J5 J6 EE];

    for j = 1:7
    
        if k == 1
    
            if j <= 6
                label = ['J' num2str(j)];
            else
                label = 'EE';
            end
    
            joint_labels(j) = text( ...
                joint_positions(1,j), ...
                joint_positions(2,j), ...
                joint_positions(3,j), ...
                label, ...
                'FontSize',10, ...
                'FontWeight','bold', ...
                'Color','k');
    
        else
    
            set(joint_labels(j), ...
                'Position',[ ...
                joint_positions(1,j), ...
                joint_positions(2,j), ...
                joint_positions(3,j)]);
    
        end
    end


    P = [J1 J2 J3 J4 J5 J6 EE];

    set(robot_handle,...
        'XData',P(1,:),...
        'YData',P(2,:),...
        'ZData',P(3,:));

    ee_path = [ee_path EE];

    set(actual_handle,...
        'XData',ee_path(1,:),...
        'YData',ee_path(2,:),...
        'ZData',ee_path(3,:));

    drawnow;
    % === CAPTURE FRAME ===
    frame = getframe(gcf);
    writeVideo(vidObj, frame);

end
close(vidObj);

disp('Video successfully saved.');
end