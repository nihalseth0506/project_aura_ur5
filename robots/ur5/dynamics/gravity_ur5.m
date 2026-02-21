function tau_g = gravity_ur5(q)

params = ur5_params();
m  = params.m;
rc = params.rc;
g  = params.g;   % should be [0;0;-9.81]

tau_g = zeros(6,1);

% Get all transforms
[T01,T02,T03,T04,T05,T06] = forward_chain_ur5(q);

T = {eye(4), T01, T02, T03, T04, T05, T06};

% Extract joint origins and z axes
o = zeros(3,7);
z = zeros(3,6);

for i = 1:7
    o(:,i) = T{i}(1:3,4);
end

for i = 1:6
    z(:,i) = T{i}(1:3,3);
end

% Loop through each link
for i = 1:6
    
    R = T{i+1}(1:3,1:3);
    
    % COM position in base frame
    p_com = T{i+1}(1:3,4) + R * rc(i,:)';
    
    Jv = zeros(3,6);
    
    % Only joints up to i affect link i
    for j = 1:i
        Jv(:,j) = cross(z(:,j), (p_com - o(:,j)));
    end
    
    % Add gravity contribution
    tau_g = tau_g + Jv' * (m(i) * g);
    
end

end