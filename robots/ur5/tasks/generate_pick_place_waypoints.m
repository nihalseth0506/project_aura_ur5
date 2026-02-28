function waypoints = generate_pick_place_waypoints(x_home)

% x_home = initial end-effector position (3x1)

% Define offsets (in meters)
pick_offset   = [0.2; 0.0; 0.0];
place_offset  = [0.0; 0.3; 0.0];
lift_height   = 0.25;

% Define Cartesian positions
x_pick   = x_home + pick_offset;
x_lift   = x_pick;   x_lift(3)  = x_lift(3) + lift_height;
x_place  = x_home + place_offset;
x_place(3) = x_pick(3);
x_place_lift = x_place; x_place_lift(3) = x_place_lift(3) + lift_height;

% Sequence
waypoints = [ ...
    x_home, ...
    x_pick, ...
    x_lift, ...
    x_place_lift, ...
    x_place, ...
    x_home ...
    ];

end