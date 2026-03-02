function animate_ur5_stick(q_traj, waypoints, payload_flag)

figure('Units','pixels',...
       'Position',[100 100 1400 900],...
       'Color','white',...
       'MenuBar','none',...
       'ToolBar','none');

axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(210,0)
hold on;

title('Simulation of Pick and Place Operation of UR5 Robot', ...
      'FontSize',16, ...
      'FontWeight','bold');

xlim([-1 1]);
ylim([-1 1]);
zlim([0 1]);

%% =============================
% VIDEO SETUP (STABLE VERSION)
%% =============================
results_path = fullfile('media','images','sprint4');
if ~exist(results_path,'dir')
    mkdir(results_path);
end

video_file = fullfile(results_path,'sprint4_animation_sideview.avi');

vidObj = VideoWriter(video_file,'Motion JPEG AVI');
vidObj.FrameRate = 40;
vidObj.Quality = 100;
open(vidObj);

%% -----------------------------
% Waypoints
%% -----------------------------
x_pick  = waypoints(:,2);
x_place = waypoints(:,5);
ee_path = [];

%% -----------------------------
% Base Geometry
%% -----------------------------
base_radius = 0.12;
base_height = 0.08;
z_shift = base_height;
[bx,by,bz] = cylinder(base_radius,50);
bz = bz * base_height;

% Side surface
surf(bx,by,bz,...
    'FaceColor',[0.2 0.2 0.2],...
    'EdgeColor',[0.7 0.7 0.7]);

% Bottom cap
fill3(base_radius*cos(linspace(0,2*pi,50)),...
      base_radius*sin(linspace(0,2*pi,50)),...
      zeros(1,50),...
      [0.15 0.15 0.15]);

% Top cap
fill3(base_radius*cos(linspace(0,2*pi,50)),...
      base_radius*sin(linspace(0,2*pi,50)),...
      base_height*ones(1,50),...
      [0.25 0.25 0.25]);
%% =========================
% TABLES (Solid Blocks)
%% =========================
function draw_table(center, size_xy, height)

x0 = center(1);
y0 = center(2);

sx = size_xy/3;
sy = size_xy/3;

% 8 vertices of cuboid
V = [
    -sx -sy 0;
     sx -sy 0;
     sx  sy 0;
    -sx  sy 0;
    -sx -sy height;
     sx -sy height;
     sx  sy height;
    -sx  sy height
];

V(:,1) = V(:,1) + x0;
V(:,2) = V(:,2) + y0;

F = [
    1 2 3 4;   % bottom
    5 6 7 8;   % top
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

patch('Vertices',V,'Faces',F,...
      'FaceColor',[0.6 0.3 0.1],...
      'EdgeColor','w');
end
table_height = x_pick(3) + z_shift - 0.02;
draw_table(x_pick,0.25,table_height);
draw_table(x_place,0.25,table_height);

%% -----------------------------
% Cube geometry
%% -----------------------------
cube_size = 0.05;
[v,f] = create_cube_geometry(cube_size);

cube_handle = patch('Vertices', v, ...
    'Faces', f, ...
    'FaceColor', [1 0.8 0], ...
    'EdgeColor', 'k');

robot_handle = plot3(0,0,0,'-o',...
    'LineWidth',4,...
    'MarkerSize',6,...
    'MarkerFaceColor',[0 0.4 0.8]);

path_handle = plot3(0,0,0,'k','LineWidth',1.5);

cube_position = x_pick + [0;0;z_shift];
attached = false;

grip_left_handle  = plot3(0,0,0,'k','LineWidth',4);
grip_right_handle = plot3(0,0,0,'k','LineWidth',4);
grip_top_handle    = plot3(0,0,0,'k','LineWidth',4);
filler_handle = plot3(0,0,0,'k','LineWidth',6);

joint_labels = gobjects(6,1);

cube_R = eye(3);

for k = 1:size(q_traj,2)

    camorbit(0.3,0,'data',[0 0 1])

    q = q_traj(:,k);
    [T01,T02,T03,T04,T05,T06] = forward_chain_ur5(q');

    p1 = T01(1:3,4);
    p2 = T02(1:3,4);
    p3 = T03(1:3,4);
    p4 = T04(1:3,4);
    p5 = T05(1:3,4);
    p6 = T06(1:3,4);

    % Shift robot above base
    z_shift = base_height;
    p1(3)=p1(3)+z_shift;
    p2(3)=p2(3)+z_shift;
    p3(3)=p3(3)+z_shift;
    p4(3)=p4(3)+z_shift;
    p5(3)=p5(3)+z_shift;
    p6(3)=p6(3)+z_shift;

    %% Physical joint motor positions
    J1 = [0;0;base_height];
    J2 = p1;
    J3 = p2;
    J4 = p3;
    J5 = p4;
    J6 = p5;
    EE = p5;

    P = [J1 J2 J3 J4 J5 J6 EE];
    ee_path = [ee_path EE];

    set(robot_handle,'XData',P(1,:),...
        'YData',P(2,:),...
        'ZData',P(3,:));

    set(path_handle,'XData',ee_path(1,:),...
        'YData',ee_path(2,:),...
        'ZData',ee_path(3,:));

    joint_positions = [J1 J2 J3 J4 J5 J6];

    for j = 1:6
        if k == 1
            joint_labels(j) = text(joint_positions(1,j),...
                joint_positions(2,j),...
                joint_positions(3,j),...
                ['J' num2str(j)],...
                'FontSize',10,...
                'FontWeight','bold',...
                'Color','k');
        else
            set(joint_labels(j),'Position',...
                [joint_positions(1,j),...
                 joint_positions(2,j),...
                 joint_positions(3,j)]);
        end
    end
    R_ee = T06(1:3,1:3);
    flange = EE;
    flange_offset = 0.02;   % small metal adapter
    plate_center = flange + R_ee*[0;0;flange_offset];
    tool_tip = plate_center - R_ee*[cube_size/2;0;0];
    
    if k == 1
        ee_label = text(tool_tip(1),tool_tip(2),tool_tip(3),'EE',...
            'FontSize',10,...
            'FontWeight','bold',...
            'Color','r');
    else
        set(ee_label,'Position',[tool_tip(1),tool_tip(2),tool_tip(3)]);
    end

    %x = p6;

    %% Realistic C-Gripper (Flange → Plate → Fingers)

    % Opening width
    if payload_flag(k)
        grip_width = cube_size * 1.0;   % closed
    else
        grip_width = cube_size * 1.8;   % open
    end
    
    R_ee = T06(1:3,1:3);
    
    % 1️⃣ Flange origin (J6 / EE)
    flange = EE;
    
    % 2️⃣ Small flange extension forward
    flange_offset = 0.02;   % small metal adapter
    plate_center = flange + R_ee*[0;0;flange_offset];
    % Draw filler (adapter between flange and plate)
    filler_end = plate_center;
    
    set(filler_handle,...
        'XData',[flange(1) filler_end(1)],...
        'YData',[flange(2) filler_end(2)],...
        'ZData',[flange(3) filler_end(3)]);

    % 3️⃣ Horizontal plate (back of C)
    plate_half = grip_width/2;
    
    plate_left_local  = [0;  plate_half; 0];
    plate_right_local = [0; -plate_half; 0];
    
    plate_left  = R_ee*plate_left_local  + plate_center;
    plate_right = R_ee*plate_right_local + plate_center;
    
    % Draw horizontal plate
    set(grip_top_handle,'XData',[plate_left(1) plate_right(1)],...
        'YData',[plate_left(2) plate_right(2)],...
        'ZData',[plate_left(3) plate_right(3)]);
    
    % 4️⃣ Vertical fingers (downward from plate)
    finger_length = cube_size;

    % Extend fingers outward along local X (NOT Z)
    left_bottom  = plate_left;
    left_tip     = plate_left  + R_ee*[0;0;finger_length];
    
    right_bottom = plate_right;
    right_tip    = plate_right + R_ee*[0;0;finger_length];
    
    set(grip_left_handle,'XData',[left_bottom(1) left_tip(1)],...
        'YData',[left_bottom(2) left_tip(2)],...
        'ZData',[left_bottom(3) left_tip(3)]);
    
    set(grip_right_handle,'XData',[right_bottom(1) right_tip(1)],...
        'YData',[right_bottom(2) right_tip(2)],...
        'ZData',[right_bottom(3) right_tip(3)]);
    %% Cube logic
    
    % Detect attachment / detachment
    if payload_flag(k) && ~attached
        attached = true;
    end
    
    if ~payload_flag(k) && attached
        attached = false;
    
        % --- Make cube flat but keep yaw ---
        yaw = atan2(cube_R(2,1), cube_R(1,1));
    
        cube_R = [ cos(yaw) -sin(yaw) 0;
                   sin(yaw)  cos(yaw) 0;
                   0         0        1 ];
    
        % Adjust Z so cube sits flat on table
        
        cube_position(3) = table_height + cube_size/2;
    end
    
    if attached
        % Follow end-effector
        T = T06;
        cube_R = T(1:3,1:3);
        cube_position = T06(1:3,4);
        cube_position(3) = cube_position(3) + z_shift;
    end
    
    % Always draw cube using stored pose
    v_transformed = (cube_R * v')' + cube_position';

    set(cube_handle,'Vertices',v_transformed);

    plot3(x_pick(1), x_pick(2), x_pick(3)+z_shift,...
        'go','MarkerSize',10,'LineWidth',2);

    plot3(x_place(1), x_place(2), x_place(3)+z_shift,...
        'ro','MarkerSize',10,'LineWidth',2);

    xlim([-1 1]);
    ylim([-1 1]);
    zlim([0 1]);
    
    if k == 1
        drawnow;
        pause(1);   % pause at initial pose
    end

    drawnow;
    % === CAPTURE FRAME ===
    frame = getframe(gcf);
    writeVideo(vidObj, frame);
end

close(vidObj);

legend([robot_handle path_handle cube_handle],... 
    {'Robot','End Effector Path','Object'},... 
    'Location','northeast'); 

disp('Video successfully saved.');

end
