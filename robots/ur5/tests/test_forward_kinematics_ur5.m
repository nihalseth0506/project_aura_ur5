clc;
clear;
close all;

addpath(genpath(pwd))

% Example joint configuration (in radians)
theta_vals = [0, -pi/4, pi/3, -pi/6, pi/4, 0];

T = forward_kinematics_ur5(theta_vals);

disp('T06 = ');
disp(vpa(T,4));

R = T(1:3,1:3);
det(R)
