function q = apply_joint_limits(q)

q_min = deg2rad([-180 -180 -180 -180 -180 -180])';
q_max = deg2rad([ 180  180  180  180  180  180])';

q = max(min(q, q_max), q_min);

end