clc;
clear;
close all;

% Declare symbolic variables
syms theta d a alpha real

% Call DH function
A = dh_transform(theta, d, a, alpha);

% Simplify result
A = simplify(A);

disp('DH Transformation Matrix:');
disp(A);
