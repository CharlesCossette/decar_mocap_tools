function [err, error_position, error_velocity, error_accel] = ...
    imuAccelDeadReckoningError2(x, dataSynced, params) 
% ERRORDEADRECKONING Corrects the IMU with the provided calibration
% parameters, then integrates the data forward on specific intervals.
% Returns the error between the integrated solution and the ground truth.
    C_ma = x{1};
    bias_a = x{2};
    scale_a = x{3};
    skew_a = x{4};
    C_ae = x{5};
    
    % Extract some relevant information and parameters.
    is_gap = dataSynced.gapIndices(:);
    is_static = dataSynced.staticIndices(:);   
    interval_size = params.interval_size;
    batch_size = params.batch_size;
    start_index = params.start_index;
    end_index = params.end_index;   
    
    % Calibrated Accelerometer measurements
    data_corrected.t = dataSynced.t;
    T_skew_a = eye(3);
    T_skew_a(1,2) = -skew_a(3);
    T_skew_a(1,3) =  skew_a(2);
    T_skew_a(2,3) = -skew_a(1);
    data_corrected.accel = C_ma*T_skew_a*diag(scale_a)*(dataSynced.accel + bias_a);
    
    % Corrected gravity in the mocap world frame.
    g_e = [0;0;-9.80665];
    g_a = C_ae*g_e;
    
    % Go through each interval and dead-reckon for a small duration of
    % length batch_size. Compare results to ground truth.
    error_position = nan(3,length(dataSynced.t));
    error_velocity = nan(3,length(dataSynced.t));
    error_accel = nan(3,length(dataSynced.t));
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
            r_zw_a_0 = dataSynced.r_zw_a(:,lv1);
            if is_static(lv1)
                v_zwa_a_0 = zeros(3,1);
            else
                v_zwa_a_0 = dataSynced.v_zwa_a(:,lv1);
            end
            t_span = dataSynced.t(idx);        

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
                C_ba = dataSynced.C_ba(:,:,measurement_index); % TODO CHECK INDEX.
                v_zwa_a_dr(:,lv2+1) = v_zwa_a_dr(:,lv2) + (C_ba.'*u_acc_b + g_a)*dt;
                r_zw_a_dr(:,lv2+1) = r_zw_a_dr(:,lv2) + v_zwa_a_dr(:,lv2)*dt;
                a_zwa_a_dr(:,lv2) = C_ba.'*u_acc_b + g_a;
            end

            % Build errors. If static, force zero velocity as the reference. 
            error_position(:,idx) = dataSynced.r_zw_a(:,idx) - r_zw_a_dr;
            error_velocity(:,idx) = dataSynced.v_zwa_a(:,idx) - v_zwa_a_dr;                                   
            error_accel(:,idx) = dataSynced.a_zwa_a(:,idx) - a_zwa_a_dr;

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