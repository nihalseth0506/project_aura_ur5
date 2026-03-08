function w = manipulability_ur5(q)

J = jacobian_ur5(q');

w = sqrt(det(J*J'));

end