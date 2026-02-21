function tau = inverse_dynamics_ur5(q, qdot, qddot)

M = mass_matrix_ur5(q);
G = gravity_ur5(q);
C = coriolis_ur5(q, qdot);
tau = M * qddot + C*qdot + G;
%tau = M * qddot + G;
end
