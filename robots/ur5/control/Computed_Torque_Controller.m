function tau = Computed_Torque_Controller(q,qdot,q_des,qdot_des)

Kp = 10*eye(6);
Kd = 5*eye(6);

e  = q_des - q;
ed = qdot_des - qdot;

% desired acceleration
qddot_cmd = Kp*e + Kd*ed;

% robot dynamics
M = mass_matrix_ur5(q');
C = coriolis_ur5(q',qdot');
G = gravity_ur5(q');

tau = M*qddot_cmd + C*qdot + G;

end