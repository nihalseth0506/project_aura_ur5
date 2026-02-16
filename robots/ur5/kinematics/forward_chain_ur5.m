function [T01, T02, T03, T04, T05, T06] = forward_chain_ur5(theta_vals)

% UR5 DH parameters
d1 = 0.089159;
a2 = -0.425;
a3 = -0.39225;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823;

theta1 = theta_vals(1);
theta2 = theta_vals(2);
theta3 = theta_vals(3);
theta4 = theta_vals(4);
theta5 = theta_vals(5);
theta6 = theta_vals(6);

% Individual transforms
A1 = dh_transform(theta1, d1, 0, pi/2);
A2 = dh_transform(theta2, 0, a2, 0);
A3 = dh_transform(theta3, 0, a3, 0);
A4 = dh_transform(theta4, d4, 0, pi/2);
A5 = dh_transform(theta5, d5, 0, -pi/2);
A6 = dh_transform(theta6, d6, 0, 0);

% Cumulative transforms
T01 = A1;
T02 = T01 * A2;
T03 = T02 * A3;
T04 = T03 * A4;
T05 = T04 * A5;
T06 = T05 * A6;

end
