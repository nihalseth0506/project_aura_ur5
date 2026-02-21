clc;
clear;
addpath(genpath(pwd));

q = [0; -pi/4; pi/3; -pi/6; pi/4; 0];

tau_g = gravity_ur5(q);

disp("Gravity Torque Vector:");
disp(tau_g);
