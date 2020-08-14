function [mocap_accel, mocap_gyro, data] = getFakeImuMocap(splineStruct,t,g_a)
    temp      = ppval(splineStruct,t);
    tempDerv  = splineDerv(splineStruct, t, 1);
    tempDerv2 = splineDerv(splineStruct, t, 2);

    mocap_accel   = zeros(3,length(t));
    mocap_gyro = zeros(3,length(t));
    
    for lv1=1:length(t)
        % Mocap omega data
        q_ba     = temp(4:7,lv1);
        q_ba = q_ba./norm(q_ba);
        q_ba_dot = tempDerv(4:7,lv1);
        omega_ba_b = quatrate2omega(q_ba, q_ba_dot);
        mocap_gyro(:,lv1) = omega_ba_b;

        % Mocap acceleration data
        a_zwa_a = tempDerv2(1:3,lv1);
        C_ba = quat2dcm(q_ba.');
        mocap_accel(:,lv1) = C_ba*(a_zwa_a - g_a);
    end
    
    data.t = t;
    data.r_zw_a = temp(1:3,:);
    data.q_ba = temp(4:7,:)./vecnorm(temp(4:7,:));
    data.C_ba = quat2dcm(data.q_ba.');
    data.v_zwa_a = tempDerv(1:3,:);
    data.a_zwa_a = tempDerv2(1:3,:);
end