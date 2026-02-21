function M = mass_matrix_ur5(q)

params = ur5_params();
m  = params.m;
rc = params.rc;
I  = params.I;

M = zeros(6,6);

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

for i = 1:6
    
    % Rotation of link i
    R = T{i+1}(1:3,1:3);
    
    % COM in base frame
    p_com = T{i+1}(1:3,4) + R * rc(i,:)';
    
    Jv = zeros(3,6);
    Jw = zeros(3,6);
    
    for j = 1:i
        
        Jv(:,j) = cross(z(:,j), (p_com - o(:,j)));
        Jw(:,j) = z(:,j);
        
    end
    
    % Translational contribution
    M = M + m(i) * (Jv' * Jv);
    
    % Rotational contribution
    M = M + Jw' * R * I(:,:,i) * R' * Jw;
    
end

end
