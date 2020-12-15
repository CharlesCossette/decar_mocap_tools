function [err, error_position, error_velocity, error_accel] = ...
    imuAccelDeadReckoningError(r_iz_b, C_ma, bias_a, scale_a, skew_a, C_ae,...
                               data_synced, params) 
% ERRORDEADRECKONING Corrects the IMU with the provided calibration
% parameters, then integrates the data forward on specific intervals.
% Returns the error between the integrated solution and the ground truth.
    
    % Extract some relevant information and parameters.
    is_gap = data_synced.gapIndices(:);
    is_static = data_synced.staticIndices(:);   
    interval_size = params.interval_size;
    batch_size = params.batch_size;
    start_index = params.start_index;
    end_index = params.end_index;   
    
    % Calibrated Accelerometer measurements
    data_corrected.t = data_synced.t;
    T_skew_a = eye(3);
    T_skew_a(1,2) = -skew_a(3);
    T_skew_a(1,3) =  skew_a(2);
    T_skew_a(2,3) = -skew_a(1);
    data_corrected.accel = C_ma*T_skew_a*diag(scale_a)*(data_synced.accel + bias_a);
    
    % Corrected gravity in the mocap world frame.
    g_e = [0;0;-9.80665];
    g_a = C_ae*g_e;
    
    % Set new pivot point for the mocap data only if r_iz_b is not the zero vector.
    if any(r_iz_b)
        gap_size = 1;
        data_pivoted = mocapSetNewPivotPoint(data_synced, r_iz_b);
        data_pivoted.gapIntervals...
            = getIntervalsFromIndices(data_pivoted.t, data_pivoted.gapIndices, 1, 0.25);
        mocap_spline = mocapGetSplineProperties(data_pivoted, gap_size);
        [mocap_accel, mocap_gyro, mocap_corrected]...
                = getFakeImuMocap(mocap_spline,data_pivoted.t,g_a);
        data_pivoted.v_zwa_a = mocap_corrected.v_zwa_a;
        data_pivoted.a_zwa_a = mocap_corrected.a_zwa_a;
        data_pivoted.accel_mocap = mocap_accel;
        data_pivoted.gyro_mocap = mocap_gyro;
    else
        data_pivoted = data_synced;
    end
        
    % Go through each interval and dead-reckon for a small duration of
    % length batch_size. Compare results to ground truth.
    error_position = nan(3,length(data_pivoted.t));
    error_velocity = nan(3,length(data_pivoted.t));
    error_accel = nan(3,length(data_pivoted.t));
    for lv1 = start_index:interval_size:end_index 
        if ~is_gap(lv1)
            if (lv1 + batch_size) > end_index
                N = end_index - lv1 + 1;
            else
                N = batch_size;
            end
            idx = lv1:lv1 + N -1;
            idx = idx(:);
            isStaticInBatch = idx(:).*is_static(idx);
            isStaticInBatch = isStaticInBatch(isStaticInBatch ~= 0);

            % Initial conditions from mocap (if we detected static, force zero
            % velocity).
            r_zw_a_0 = data_pivoted.r_zw_a(:,lv1);
            if is_static(lv1)
                v_zwa_a_0 = zeros(3,1);
            else
                v_zwa_a_0 = data_pivoted.v_zwa_a(:,lv1);
            end
            t_span = data_pivoted.t(idx);        

            % Initialize
            N = length(t_span);
            r_zw_a_dr = zeros(3,N);
            v_zwa_a_dr = zeros(3,N);
            a_zwa_a_dr = zeros(3,N);
            r_zw_a_dr(:,1) = r_zw_a_0;
            v_zwa_a_dr(:,1) = v_zwa_a_0;

            % Dead reckoning        
            for lv2 = 1:N-1
                dt = t_span(lv2+1) - t_span(lv2);

                % Use zero-order hold
                measurement_index = find(data_corrected.t <= t_span(lv2), 1, 'last'); 
                u_acc_b = data_corrected.accel(:,measurement_index);
                C_ba = data_pivoted.C_ba(:,:,measurement_index); % TODO CHECK INDEX.
                v_zwa_a_dr(:,lv2+1) = v_zwa_a_dr(:,lv2) + (C_ba.'*u_acc_b + g_a)*dt;
                r_zw_a_dr(:,lv2+1) = r_zw_a_dr(:,lv2) + v_zwa_a_dr(:,lv2)*dt;
                a_zwa_a_dr(:,lv2) = C_ba.'*u_acc_b + g_a;
            end

            % Build errors. If static, force zero velocity as the reference. 
            error_position(:,idx) = data_pivoted.r_zw_a(:,idx) - r_zw_a_dr;
            error_velocity(:,idx) = data_pivoted.v_zwa_a(:,idx) - v_zwa_a_dr;                                   
            error_accel(:,idx) = data_pivoted.a_zwa_a(:,idx) - a_zwa_a_dr;

            error_velocity(:,isStaticInBatch) = -v_zwa_a_dr(:,isStaticInBatch - lv1 + 1);
            error_accel(:,isStaticInBatch) = -a_zwa_a_dr(:,isStaticInBatch - lv1 + 1);
        end
    end
    
    % Add weight when static. 
    error_position(:,is_static) = 10*error_position(:,is_static);
    error_velocity(:,is_static) = 10*error_velocity(:,is_static);
    error_accel(:,is_static) = 10*error_accel(:,is_static);  
    
    % Discard any corresponding to gaps in the mocap data, as the spline is
    % potentially inaccurate during these times. 
    error_position = error_position(:,~is_gap);
    error_velocity = error_velocity(:,~is_gap);
    error_accel = error_accel(:,~is_gap);
    
    % Remove any NANs, which are periods for which we are not
    % dead-reckoning.
    % Warning: potential silent failing here if the indexing is not done
    % properly.... NANs will just get deleted.
    error_position = error_position(:,all(~isnan(error_position),1));
    error_velocity = error_velocity(:,all(~isnan(error_velocity),1));
    error_accel = error_accel(:,all(~isnan(error_accel),1));


    err = [
           error_position(:);
           0.1*error_velocity(:);
           0.1*error_accel(:);
           ];
        
end