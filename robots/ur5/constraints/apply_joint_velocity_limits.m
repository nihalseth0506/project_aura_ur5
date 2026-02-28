function qdot = apply_joint_velocity_limits(qdot)

qdot_max = [2;2;2;2;2;2];  % rad/s (example)

qdot = max(min(qdot, qdot_max), -qdot_max);

end