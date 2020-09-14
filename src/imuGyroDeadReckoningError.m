function [err, error_attitude] = ...
    imuGyroDeadReckoningError(C_mg, bias_g, scale_g, skew_g, dataSynced, params)
% ERRORDEADRECKONING Corrects the IMU with the provided calibration
% parameters, then integrates the data forward on specific intervals.
% Returns the error between the integrated solution and the ground truth.
    
    % Extract some relevant information and parameters.
    is_gap = dataSynced.gapIndices(:);
    is_static = dataSynced.staticIndices(:);   
    interval_size = params.interval_size;
    batch_size = params.batch_size;
    start_index = params.start_index;
    end_index = params.end_index;
        
    data_corrected.t = dataSynced.t;
    
    % Calibrated Gyroscope measurements
    T_skew_g = eye(3);
    T_skew_g(1,2) = -skew_g(3);
    T_skew_g(1,3) =  skew_g(2);
    T_skew_g(2,3) = -skew_g(1);
    data_corrected.gyro = C_mg*T_skew_g*diag(scale_g)*(dataSynced.gyro + bias_g);

    % Go through each interval and dead-reckon for a small duration of
    % length batch_size. Compare results to ground truth.
    error_attitude = nan(3,length(dataSynced.t));
    error_omega = nan(3,length(dataSynced.t));
    for lv1 = start_index:interval_size:end_index 
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
        C_ba_0 = dataSynced.C_ba(:,:,lv1);
        t_span = dataSynced.t(idx);
        
        % Initialize
        N = length(t_span);
        C_ba_dr = zeros(3,3,N);
        C_ba_dr(:,:,1) = C_ba_0;
        
        % Dead reckoning        
        for lv2 = 1:N-1
            dt = t_span(lv2+1) - t_span(lv2);
            % Use zero-order hold. TODO CHECK THIS IS NOT GETTING MESSED UP
            measurement_index = find(data_corrected.t <= t_span(lv2), 1, 'last');    
            omega_ba_b = data_corrected.gyro(:,measurement_index);
            C_ba_dr(:,:,lv2+1) = ROTVEC_TO_DCM(omega_ba_b*dt)*C_ba_dr(:,:,lv2);
        end
        
        % Normalize to avoid some numerical errors.
        %C_ba_dr = quatToDcm(dcmToQuat(C_ba_dr));
        
        % Build errors. If static, force zero ang. velocity as the reference. 
        error_omega(:,idx) = dataSynced.gyro_mocap(:,idx) - data_corrected.gyro(:,idx);
                                   
        error_omega(:,isStaticInBatch) = - data_corrected.gyro(:,isStaticInBatch);
                                
        error_attitude(:,idx) = DCM_TO_ROTVEC(matmul3d(C_ba_dr,...
                                              trans3d(dataSynced.C_ba(:,:,idx))));
    end
    
    % Add weight when static. 
    error_attitude(:,is_static) = 10*error_attitude(:,is_static);
    
    % Discard any corresponding to gaps in the mocap data, as the spline is
    % potentially inaccurate during these times. 
    error_attitude = error_attitude(:,~is_gap);
    
    % Remove any NANs, which are periods for which we are not
    % dead-reckoning.
    % Warning: potential silent failing here if the indexing is not done
    % properly.... NANs will just get deleted.
    error_attitude = error_attitude(:,all(~isnan(error_attitude),1));
    error_omega = error_omega(:,all(~isnan(error_omega),1));
              
    err = [
           error_attitude(:);
           0.1*error_omega(:);
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