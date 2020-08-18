function [error, error_position, error_velocity, error_attitude] = ...
    imuDeadReckoningError(...
    C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae,...
    dataSynced, params ...
    ) 
% ERRORDEADRECKONING Corrects the IMU with the provided calibration
% parameters, then integrates the data forward on specific intervals.
% Returns the error between the integrated solution and the ground truth.
    
    % Extract some relevant information and parameters.
    isGap = dataSynced.gapIndices(:);
    isStatic = dataSynced.staticIndices(:);   
    interval_size = params.interval_size;
    batch_size = params.batch_size;
    start_index = params.start_index;
    end_index = params.end_index;
    
    calib_params.C_ms_accel = C_ma;
    calib_params.C_ms_gyro = C_mg;
    calib_params.bias_accel = bias_a;
    calib_params.bias_gyro = bias_g;
    calib_params.scale_accel = scale_a;
    calib_params.scale_gyro = scale_g;
    calib_params.skew_accel = skew_a;
    calib_params.skew_gyro = skew_g;
   
    data_corrected = imuCorrectMeasurements(dataSynced, calib_params);
    
    % Corrected gravity in the mocap world frame.
    g_e = [0;0;-9.80665];
    g_a = C_ae*g_e;
    
    % Go through each interval and dead-reckon for a small duration of
    % length batch_size. Compare results to ground truth.
    error_position = nan(3,length(dataSynced.t));
    error_velocity = nan(3,length(dataSynced.t));
    error_attitude = nan(3,length(dataSynced.t));
    error_accel = nan(3,length(dataSynced.t));
    error_omega = nan(3,length(dataSynced.t));
    for lv1 = start_index:interval_size:end_index 
        if (lv1 + batch_size) > end_index
            N = end_index - lv1 + 1;
        else
            N = batch_size;
        end
        idx = lv1:lv1 + N -1;
        idx = idx(:);
        isStaticInBatch = idx(:).*isStatic(idx);
        isStaticInBatch = isStaticInBatch(isStaticInBatch ~= 0);
        
        % Initial conditions from mocap (if we detected static, force zero
        % velocity).
        r_zw_a_0 = dataSynced.r_zw_a(:,lv1);
        if isStatic(lv1)
            v_zwa_a_0 = zeros(3,1);
        else
            v_zwa_a_0 = dataSynced.v_zwa_a(:,lv1);
        end
        C_ba_0 = dataSynced.C_ba(:,:,lv1);
        t_span = dataSynced.t(idx);
        traj_dr = imuDeadReckoning(data_corrected, r_zw_a_0, v_zwa_a_0, C_ba_0,...
                                   g_a, 'so3',t_span);
        
        % Build errors. If static, force zero velocity as the reference. 
        error_position(:,idx) = dataSynced.r_zw_a(:,idx) - traj_dr.r_zw_a;
        error_velocity(:,idx) = dataSynced.v_zwa_a(:,idx) - traj_dr.v_zwa_a;                                   
        error_accel(:,idx) = dataSynced.a_zwa_a(:,idx) - traj_dr.a_zwa_a;
        error_omega(:,idx) = dataSynced.gyro_mocap(:,idx) - data_corrected.gyro(:,idx);
        
        error_velocity(:,isStaticInBatch) = error_velocity(:,isStaticInBatch) ...
                                       - dataSynced.v_zwa_a(:,isStaticInBatch);
        error_accel(:,isStaticInBatch) = error_accel(:,isStaticInBatch) ...
                                       - dataSynced.a_zwa_a(:,isStaticInBatch);
        error_omega(:,isStaticInBatch) = error_omega(:,isStaticInBatch) ...
                                       - dataSynced.gyro_mocap(:,isStaticInBatch);
                                   
        error_attitude(:,idx) = DCM_TO_ROTVEC(matmul3d(traj_dr.C_ba,...
                                              trans3d(dataSynced.C_ba(:,:,idx))));

    end
    
    % Add weight when static. 
    error_position(:,isStatic) = 10*error_position(:,isStatic);
    error_velocity(:,isStatic) = 10*error_velocity(:,isStatic);
    error_attitude(:,isStatic) = 10*error_attitude(:,isStatic);
    
    
    % Discard any corresponding to gaps in the mocap data, as the spline is
    % potentially inaccurate during these times. 
    error_position = error_position(:,~isGap);
    error_velocity = error_velocity(:,~isGap);
    error_attitude = error_attitude(:,~isGap);
    error_accel = error_accel(:,~isGap);
    error_omega = error_omega(:,~isGap);
    
    % Velocity ONLY when static. Trying to use the spline as little as
    % possible.
    % error_velocity = error_velocity(:,isStatic);    
    
    % Remove any NANs, which are periods for which we are not
    % dead-reckoning.
    % Warning: potential silent failing here if the indexing is not done
    % properly.... NANs will just get deleted.
    error_position = error_position(:,all(~isnan(error_position),1));
    error_velocity = error_velocity(:,all(~isnan(error_velocity),1));
    error_attitude = error_attitude(:,all(~isnan(error_attitude),1));
    error_accel = error_accel(:,all(~isnan(error_accel),1));
    error_omega = error_omega(:,all(~isnan(error_omega),1));
    
%     phi_ma = DCM_TO_ROTVEC(C_ma);
%     phi_mg = DCM_TO_ROTVEC(C_mg);
%     error_regularize = [
%                         phi_ma;
%                         phi_mg;
%                         0.1*bias_a;
%                         bias_g;
%                         ones(3,1) - scale_a;
%                         ones(3,1) - scale_g;
%                         skew_a;
%                         skew_g
%                        ];
                        
                    
    error = [
             error_position(:);
             error_velocity(:);
             10*error_attitude(:);
             0.1*error_accel(:);
             0.1*error_omega(:);
             %sqrt(size(error_position,2))*error_regularize
             ];
end

function C = matmul3d(A,B)
C = zeros(size(A,1),size(B,2), size(A,3));
assert(size(A,3) == size(B,3));
for lv1 = 1:size(A,3)
   C(:,:,lv1) = A(:,:,lv1)*B(:,:,lv1);
end

end
function A_trans = trans3d(A)
A_trans = permute(A,[2 1 3]);
end