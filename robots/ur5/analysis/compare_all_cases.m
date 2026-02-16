clc;
clear;

addpath(genpath(pwd));

% Safe configuration
q_safe = [0, -pi/4, pi/3, -pi/6, pi/4, 0]';

% Singular configuration
q_sing = [0 0 0 0 0 0]';

disp("Running experiments...");

% 1️⃣ Safe + Fixed
res1 = run_circle_experiment(q_safe, 'fixed');

% 2️⃣ Singular + Fixed
res2 = run_circle_experiment(q_sing, 'fixed');

% 3️⃣ Safe + Adaptive
res3 = run_circle_experiment(q_safe, 'adaptive');

% 4️⃣ Singular + Adaptive
res4 = run_circle_experiment(q_sing, 'adaptive');

fprintf("\n===== RESULTS =====\n");

fprintf("Safe + Fixed:\n");
disp(res1);

fprintf("Singular + Fixed:\n");
disp(res2);

fprintf("Safe + Adaptive:\n");
disp(res3);

fprintf("Singular + Adaptive:\n");
disp(res4);
