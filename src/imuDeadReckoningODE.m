function x_dot = imuDeadReckoningODE(t,x, t_meas, accel_imu, gyro_imu, g_a)
    r_zw_a = x(1:3);
    v_zwa_a = x(4:6);
    q_ba = x(7:10);
    
    % Use zero-order hold
    measurement_index = find(t_meas <= t, 1, 'last');
    a_zwa_b = accel_imu(:,measurement_index);
    omega_ba_b = gyro_imu(:,measurement_index);
    
    C_ba = quatToDcm(q_ba);
    
    r_zw_a_dot = v_zwa_a;
    v_zwa_a_dot = C_ba.'*(a_zwa_b) + g_a;
    q_ba_dot = omegaToQuatRate(q_ba, omega_ba_b);
    x_dot = [r_zw_a_dot;v_zwa_a_dot;q_ba_dot];
   
end