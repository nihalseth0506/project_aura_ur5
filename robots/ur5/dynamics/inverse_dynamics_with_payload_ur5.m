function tau = inverse_dynamics_with_payload_ur5(q, qdot, qddot, payload_mass)

% Base robot torque
tau_robot = inverse_dynamics_ur5(q, qdot, qddot);

if payload_mass > 0
    
    % End-effector Jacobian (linear part)
    J = jacobian_ur5(q');
    Jv = J(1:3,:);
    
    g = [0; 0; -9.81];
    
    % Gravity force of payload
    F_payload = payload_mass * g;
    
    % Map force to joint torques
    tau_payload = Jv' * F_payload;
    
    tau = tau_robot + tau_payload;
    
else
    tau = tau_robot;
end

end