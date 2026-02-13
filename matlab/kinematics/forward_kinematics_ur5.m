function T06 = forward_kinematics_ur5(theta_vals)
% Computes full forward kinematics of UR5

% UR5 DH parameters (constants)
d1 = 0.089159;
a2 = -0.425;
a3 = -0.39225;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823;

if nargin == 0
    % Symbolic mode
    syms theta1 theta2 theta3 theta4 theta5 theta6 real
    
    A1 = dh_transform(theta1, d1, 0, pi/2);
    A2 = dh_transform(theta2, 0, a2, 0);
    A3 = dh_transform(theta3, 0, a3, 0);
    A4 = dh_transform(theta4, d4, 0, pi/2);
    A5 = dh_transform(theta5, d5, 0, -pi/2);
    A6 = dh_transform(theta6, d6, 0, 0);

    T06 = simplify(A1*A2*A3*A4*A5*A6);

else
    % Numeric mode
    theta1 = theta_vals(1);
    theta2 = theta_vals(2);
    theta3 = theta_vals(3);
    theta4 = theta_vals(4);
    theta5 = theta_vals(5);
    theta6 = theta_vals(6);

    A1 = dh_transform(theta1, d1, 0, pi/2);
    A2 = dh_transform(theta2, 0, a2, 0);
    A3 = dh_transform(theta3, 0, a3, 0);
    A4 = dh_transform(theta4, d4, 0, pi/2);
    A5 = dh_transform(theta5, d5, 0, -pi/2);
    A6 = dh_transform(theta6, d6, 0, 0);

    T06 = A1*A2*A3*A4*A5*A6;
end

end
