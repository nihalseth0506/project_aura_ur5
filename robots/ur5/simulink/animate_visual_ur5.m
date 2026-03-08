clc
clear
close all

simOut = sim('ur5_controller');

q_log = squeeze(simOut.q_log);
xd_log = squeeze(simOut.xd_log);

q_traj = q_log;
xd_traj = xd_log;

animate_ur5_simulink(q_traj, xd_traj)