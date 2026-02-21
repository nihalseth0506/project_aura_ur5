clc;
clear;
addpath(genpath(pwd));

q = [0; -pi/4; pi/3; -pi/6; pi/4; 0];

M = mass_matrix_ur5(q);

disp("Mass Matrix M(q):");
disp(M);

% Check symmetry
disp("Symmetry check (M - M'):");
disp(M - M');

% Check positive definiteness
eigvals = eig(M);
disp("Eigenvalues:");
disp(eigvals);

[V,D] = eig(M);

% Sort eigenvalues descending
[lambda_sorted, idx] = sort(diag(D),'descend');

v_heaviest = V(:, idx(1));
