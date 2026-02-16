function J = jacobian_ur5(theta_vals)

% Get transformation matrices step-by-step
[T01, T02, T03, T04, T05, T06] = forward_chain_ur5(theta_vals);

% Origins
o0 = [0;0;0];
o1 = T01(1:3,4);
o2 = T02(1:3,4);
o3 = T03(1:3,4);
o4 = T04(1:3,4);
o5 = T05(1:3,4);
o6 = T06(1:3,4);

% Z axes
z0 = [0;0;1];
z1 = T01(1:3,3);   
z2 = T02(1:3,3);
z3 = T03(1:3,3);
z4 = T04(1:3,3);
z5 = T05(1:3,3);

% Linear velocity part
Jv1 = cross(z0, (o6 - o0));
Jv2 = cross(z1, (o6 - o1));
Jv3 = cross(z2, (o6 - o2));
Jv4 = cross(z3, (o6 - o3));
Jv5 = cross(z4, (o6 - o4));
Jv6 = cross(z5, (o6 - o5));

% Angular velocity part
Jw1 = z0;
Jw2 = z1;
Jw3 = z2;
Jw4 = z3;
Jw5 = z4;
Jw6 = z5;

% Assemble Jacobian
J = [Jv1 Jv2 Jv3 Jv4 Jv5 Jv6;
     Jw1 Jw2 Jw3 Jw4 Jw5 Jw6];

end
