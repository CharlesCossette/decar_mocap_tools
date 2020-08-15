function trajectory = imuDeadReckoning(dataIMU, r_zw_a_0, v_zwa_a_0, C_ba_0, g_a, method)

switch method
    case 'euler'
        % Initial conditions
        q_ba_0 = smoothdcm2quat(C_ba_0);
        x_0 = [r_zw_a_0; v_zwa_a_0; q_ba_0.'];
        
        % Time range to integrate over
        t_span = dataIMU.t;
        
        % Continuous-time imu dead reckoning equations
        f = @(t,x) imuDeadReckoningODE(t,x, dataIMU.t, dataIMU.accel, ...
            dataIMU.gyro, g_a);
        
        % Option to normalize quaternions as we integrate.
        options.indices_to_normalize = 7:10;
        [~, x_euler] = ode1(f, t_span, x_0, options);
        
        trajectory.r_zw_a = x_euler(:,1:3).';
        trajectory.v_zwa_a = x_euler(:,4:6).';
        trajectory.q_ba = x_euler(:,7:10).';
    case 'rk4'
        % Initial conditions
        q_ba_0 = smoothdcm2quat(C_ba_0);
        x_0 = [r_zw_a_0; v_zwa_a_0; q_ba_0.'];
        
        % Time range to integrate over
        t_span = dataIMU.t;
        
        % Continuous-time imu dead reckoning equations
        f = @(t,x) imuDeadReckoningODE(t,x, dataIMU.t, dataIMU.accel, ...
            dataIMU.gyro, g_a);
        
        % Option to normalize quaternions as we integrate.
        options.indices_to_normalize = 7:10;
        [~, x_euler] = ode4(f, t_span, x_0, options);
        
        trajectory.r_zw_a = x_euler(:,1:3).';
        trajectory.v_zwa_a = x_euler(:,4:6).';
        trajectory.q_ba = x_euler(:,7:10).';
    case 'so3'
        % Initialize
        N = length(dataIMU.t);
        r_zw_a_so3 = zeros(3,N);
        v_zwa_a_so3 = zeros(3,N);
        C_ba_so3 = zeros(3,3,N);
        a_zwa_a_so3 = zeros(3,N);
        r_zw_a_so3(:,1) = r_zw_a_0;
        v_zwa_a_so3(:,1) = v_zwa_a_0;
        C_ba_so3(:,:,1) = C_ba_0;
        
        % Dead reckoning
        
        for lv1 = 1:N-1
            dt = dataIMU.t(lv1+1) - dataIMU.t(lv1);
            omega_ba_b = dataIMU.gyro(:,lv1);
            a_zwa_b = dataIMU.accel(:,lv1);
            C_ba_so3(:,:,lv1+1) = expm(-CrossOperator(omega_ba_b*dt))*C_ba_so3(:,:,lv1);
            v_zwa_a_so3(:,lv1+1) = v_zwa_a_so3(:,lv1) + (C_ba_so3(:,:,lv1).'*a_zwa_b + g_a)*dt;
            r_zw_a_so3(:,lv1+1) = r_zw_a_so3(:,lv1) + v_zwa_a_so3(:,lv1)*dt;
            a_zwa_a_so3(:,lv1) = C_ba_so3(:,:,lv1).'*a_zwa_b + g_a;
        end
        trajectory.r_zw_a = r_zw_a_so3;
        trajectory.v_zwa_a = v_zwa_a_so3;
        trajectory.C_ba = C_ba_so3;
        trajectory.a_zwa_a = a_zwa_a_so3;
end
end
