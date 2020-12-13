function [data_synced, offset] = imuMocapSync(spline_struct, data_imu, accel_threshold, varargin)
% Synchronizes the Mocap data represented as a spline and the IMU data.
% Detects first motion in both the mocap and IMU data, and uses this as an
% initial guess for a least-squares refinement procedure. The initial guess
% can be specified using other methods. The input should be for one rigid body 
% and one IMU.  This code expressed the IMU timestamps in the MOCAP CLOCK.
%
% EXAMPLE USES:
%
%     data_synced = imuMocapSync(spline_struct, data_imu)
%     data_synced = imuMocapSync(spline_struct, data_imu, 'accel_threshold', 12)
%     data_synced = imuMocapSync(spline_struct, data_imu, 'offset_guess', 1.5)
%     data_synced = imuMocapSync(spline_struct, data_imu, 'force_sync', true)
%
% PARAMETERS:
% -----------
%
% RETURNS:
% --------
%
% Requires CURVE FITTING TOOLBOX.
% Requires OPTIMIZATION TOOLBOX.
% TODO: should be done based on first motion instead of spike. 

    if nargin < 2
        error('Missing data')
    end

    % Default options for optional arguments
    default_force_sync = false;
    default_offset_guess = false;
    default_accel_threshold = false;
    
    % Parse input for name-value pairs
    p = inputParser;
    addRequired(p,'spline_struct');
    addRequired(p,'data_imu');
    addParameter(p,'accel_threshold',default_accel_threshold);
    addParameter(p,'force_sync', default_force_sync);
    addParameter(p,'offset_guess', default_offset_guess);
    
    % Load input into variables.
    parse(p, spline_struct, data_imu, accel_threshold, varargin{:})
    spline_struct = p.Results.spline_struct;
    data_imu = p.Results.data_imu;
    accel_threshold = p.Results.accel_threshold;
    offset_guess = p.Results.offset_guess;
    force_sync = p.Results.force_sync;    
    
    g_a = [0;0;-9.80665]; % gravity 
    

    %% Generating Mocap accelerometer data using fitted spline    
    t_mocap = spline_struct.breaks;
    [mocap_acc, ~] = getFakeImuMocap(spline_struct,t_mocap,g_a);
    gap_indices = getIndicesFromIntervals(t_mocap, spline_struct.gapIntervals);
    mocap_acc(:, gap_indices) = 0;
    
    %% Calculate initial guess
    % A positive offset indicates that the IMU data starts AFTER the mocap
    % data.
    if offset_guess
        % Use manually-provided initial guess.
        offset = offset_guess;
    elseif accel_threshold
        % Default initial guess technique (accel_threshold)
        % Finding the spike in the IMU data
        accNormIMU    = vecnorm(data_imu.accel);
        spikeIndexIMU = find(accNormIMU > accel_threshold, 1, 'first');

        % Finding the spike in the Mocap data
        accNormMocap    = vecnorm(mocap_acc);
        spikeIndexMocap = find(accNormMocap > accel_threshold, 1, 'first');   

        if isempty(spikeIndexIMU) || isempty(spikeIndexMocap)
            warning(['DECAR_MOCAP_TOOLS: Spike not detected in IMU or Mocap!',...
                    'we will assume the data starts at almost the same time.'])
            tSyncIMU = data_imu.t(1);
            tSyncMocap = t_mocap(1);
        else
            tSyncIMU = data_imu.t(spikeIndexIMU);  % time of spike in imu clock
            tSyncMocap = t_mocap(spikeIndexMocap); % time of spike in mocap clock.
        end

        % the offset, to be used to extract Mocap data for the UWB data as well
        % if offset is positive, this means the mocap detected the spike later
        % (in its clock) than the IMU. This means that the IMU was started
        % afterwards, and thus the mocap is AHEAD of the IMU.
        offset = tSyncMocap - tSyncIMU;
    else
        error('DECAR_MOCAP_TOOLS: TODO: Static detection technique under development.')
    end                                        
    %% Generating the synced data
    % Move the IMU clock to the mocap clock.
    if ~force_sync
        t_synced = data_imu.t + offset; % Shift up IMU time to match mocap time.
    end
    
    % Delete IMU data that doesnt have ground truth.
    isOutsideMocap = (t_synced < 0) | (t_synced > t_mocap(end));
    t_synced = t_synced(~isOutsideMocap);
    imu_acc_synced = data_imu.accel(:,~isOutsideMocap);
    imu_gyr_synced = data_imu.gyro(:,~isOutsideMocap);
    imu_mag_synced = data_imu.mag(:,~isOutsideMocap);

    % Refine further using least-squares
    % TODO: make this step optional, as it takes a decent amount of time.
    % TODO: add scaling factor? t_synced_refined = scale*(t_synced + dt);
    if ~force_sync
        f = @(dt) error(dt, t_synced, spline_struct, imu_acc_synced, imu_gyr_synced);
        options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt',...
                               'display','iter-detailed','steptolerance',1e-8,...
                               'InitDamping',0);
        dt = lsqnonlin(f,0,[],[],options);

        % Apply results to the new clock.
        t_synced = t_synced + dt;
        
        % Total offset such that t_synced = data_imu.t + offset;
        offset = offset + dt; 
    end
    
    [mocap_acc_synced, mocap_omega_synced, data_synced] = ...
                              getFakeImuMocap(spline_struct, t_synced, g_a);
    
    %% Visualizing the time synchronization
    figure
    plot(t_synced, vecnorm(imu_acc_synced))
    hold on
    plot(t_synced, vecnorm(mocap_acc_synced))
    hold off
    grid on
    xlabel('$x$ [s]','interpreter','latex')
    ylabel('$a^{zw/a/a}$ [m/s$^2$]','interpreter','latex')
    legend('IMU Data','Mocap Data')
    title('Acceleration')
    
    figure
    plot(t_synced, vecnorm(imu_gyr_synced))
    hold on
    plot(t_synced, vecnorm(mocap_omega_synced))
    hold off   
    xlabel('$t$ [s]','interpreter','latex')
    ylabel('$\omega^{ba}_b$','interpreter','latex')
    grid on
    legend('IMU Data','Mocap Data')
    title('Angular Velocity')
    
    %% Save the synchronized data
    data_synced.t_0 = data_imu.t_0;
    data_synced.t = t_synced;
    data_synced.accel = imu_acc_synced;
    data_synced.gyro = imu_gyr_synced;
    data_synced.mag = imu_mag_synced;
    data_synced.accel_mocap = mocap_acc_synced;
    data_synced.gyro_mocap = mocap_omega_synced;
    data_synced.gapIndices = getIndicesFromIntervals(data_synced.t, spline_struct.gapIntervals);
    data_synced.staticIndices = getIndicesFromIntervals(data_synced.t, spline_struct.staticIntervals);
end

function output = error(dt, t_synced, splineStruct, imu_accel, imu_gyro)
    idx = 1:5:numel(t_synced); % Downsample for speed.
    g_a = [0;0;-9.80665];
    [mocap_acc, mocap_gyro, ~] = getFakeImuMocap(splineStruct, t_synced(idx) + dt, g_a);
    gap_indices = getIndicesFromIntervals(t_synced(idx) + dt, splineStruct.gapIntervals);
    static_indices = getIndicesFromIntervals(t_synced(idx) + dt, splineStruct.staticIntervals);
    error_accel = vecnorm(mocap_acc) - vecnorm(imu_accel(:,idx));
    error_gyro = vecnorm(mocap_gyro) - vecnorm(imu_gyro(:,idx));
    error_accel(:,gap_indices) = 0;
    error_gyro(:,gap_indices) = 0;
    error_accel(:,static_indices) = 0;
    error_gyro(:,static_indices) = 0;
    output = [0*error_accel(:); error_gyro(:)];
end