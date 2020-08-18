function omega_ba_b = quatRateToOmega(q_ba, q_ba_dot)
eta = q_ba(1);
epsilon = q_ba(2:4);
S = [-2*epsilon, 2*(eta*eye(3) - CrossOperator(epsilon))];
omega_ba_b = S*q_ba_dot;
end