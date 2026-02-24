function [energy_total, rms_torque, rms_power] = compute_energy_metrics(results)

q_hist    = results.q_history;
qdot_hist = results.qdot_history;
dt        = results.dt;
steps     = results.steps;

% Numerical acceleration
qddot_hist = zeros(6,steps);

for k = 2:steps
    qddot_hist(:,k) = (qdot_hist(:,k) - qdot_hist(:,k-1)) / dt;
end

tau_hist = zeros(6,steps);
power    = zeros(steps,1);

for k = 1:steps
    
    q     = q_hist(:,k);
    qdot  = qdot_hist(:,k);
    qddot = qddot_hist(:,k);
    
    tau = inverse_dynamics_ur5(q, qdot, qddot);
    
    tau_hist(:,k) = tau;
    power(k)      = tau' * qdot;

end

tau_norm = vecnorm(tau_hist);

rms_torque  = rms(tau_norm);
rms_power   = rms(power);
energy_total = trapz(power) * dt;

end