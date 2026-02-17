function results = run_circle_experiment(q_init, mode)

% mode = 'fixed' OR 'adaptive'

% Get initial pose
[~,~,~,~,~,T] = forward_chain_ur5(q_init');
x0 = T(1:3,4);

% Circle parameters
r = 0.03;
omega = 2*pi/5;
dt = 0.01;
total_time = 5;
steps = total_time/dt;

% Control gain
K = 5;

% Damping parameters
epsilon = 1e-6;
k_damp = 5e-5;
lambda_min = 0.01;
lambda_max = 0.5;
lambda_fixed = 0.1;

% Initialize
q = q_init;
error_history = zeros(steps,1);
qdot_norm = zeros(steps,1);
manip_history = zeros(steps,1);
trajectory = zeros(steps,3);

for k = 1:steps
    
    t = k*dt;
    
    % Desired circular trajectory
    xd = [ x0(1) + r*cos(omega*t);
           x0(2) + r*sin(omega*t);
           x0(3) ];
    
    % Current pose
    [~,~,~,~,~,T] = forward_chain_ur5(q');
    x = T(1:3,4);
    
    % Error
    e = xd - x;
    xdot = K * e;
    
    % Jacobian
    J = jacobian_ur5(q');
    
    % Manipulability (SVD)
    s = svd(J);
    w = prod(s);
    manip_history(k) = w;
    
    % Choose damping mode
    if strcmp(mode,'fixed')
        lambda = lambda_fixed;
    else
        lambda = lambda_min + k_damp/(w + epsilon);
        lambda = min(lambda, lambda_max);
    end
    
    % DLS
    qdot = J' * ((J*J' + lambda^2 * eye(6)) \ [xdot; 0;0;0]);
    
    % Integrate
    q = q + qdot * dt;
    
    % Store
    error_history(k) = norm(e);
    qdot_norm(k) = norm(qdot);
    trajectory(k,:) = x';

end

% Return full data for plotting
results.error_history = error_history;
results.qdot_norm     = qdot_norm;
results.manip_history = manip_history;
results.trajectory    = trajectory;

% Summary metrics
results.max_error  = max(error_history(100:end));
results.max_qdot   = max(qdot_norm);
results.mean_manip = mean(manip_history);

% Store parameters for external plotting
results.dt    = dt;
results.steps = steps;
results.r     = r;
results.omega = omega;
results.x0    = x0;


end
