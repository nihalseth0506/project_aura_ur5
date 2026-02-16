clc;
clear;
close all;

theta_vals = [0, -pi/4, pi/3, -pi/6, pi/4, 0];

J = jacobian_ur5(theta_vals);

disp("Jacobian:");
disp(vpa(J,4));

rank(J)

