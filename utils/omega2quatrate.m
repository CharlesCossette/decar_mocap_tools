function q_ba_dot = omega2quatrate(q_ba, omega_ba_b)
    eta = q_ba(1);
    epsilon = q_ba(2:4);
    
    gamma = 0.5*[-epsilon.';eta*eye(3) + CrossOperator(epsilon)];
    
    %q_ba_dot = gamma*omega_ba_b;
    
    ox = omega_ba_b(1);
    oy = omega_ba_b(2);
    oz = omega_ba_b(3);
    quat_skew_matrix= [  0, -ox, -oy, -oz;
                        ox,   0,  oz, -oy;
                        oy, -oz,   0,  ox;
                        oz,  oy, -ox,   0];
    q_ba_dot = 0.5*quat_skew_matrix*q_ba;
end