function [dataSynced, offset] = syncTime(splineStruct, dataIMU, options)
% Synchronizes the Mocap data represented as a spline and the IMU data,
% based on a spike in acceleration readings.
% The input should be for one rigid body and one IMU.
% This code expressed the IMU timestamps in the MOCAP CLOCK.
% Requires CURVE FITTING TOOLBOX.
% Requires OPTIMIZATION TOOLBOX.
% TODO: should be done based first motion instead of spike. 

    if nargin < 2
        error('Missing data')
    end
    if nargin < 3
        options = struct();
    end

    % Overwrite default if specified
    if exist('options','var')
        if isfield(options,'acc_threshold')
            acc_threshold = options.acc_threshold;
        else
            acc_threshold = 12;
        end
        if isfield(options,'manual_offset')
            manual_offset = options.manual_offset;
        else
            manual_offset = 0;
        end
        if isfield(options,'force_sync')
            force_sync = options.force_sync;
        else
            force_sync = false;
        end
    end
        
    
    g_a = [0;0;-9.80665]; % gravity 
    
    t_mocap = splineStruct.breaks;
    
    %% Generating Mocap accelerometer data using fitted spline
    [mocap_acc, ~] = getFakeImuMocap(splineStruct,t_mocap,g_a);
    gap_indices = getIndicesFromIntervals(t_mocap, splineStruct.gapIntervals);
    mocap_acc(:, gap_indices) = 0;
    
    %% Search for first time accel exceeds a threshold as an initial guess.
    % Finding the spike in the IMU data
    accNormIMU    = vecnorm(dataIMU.accel);
    spikeIndexIMU = find(accNormIMU > acc_threshold, 1, 'first');
                                                       
    % Finding the spike in the Mocap data
    accNormMocap    = vecnorm(mocap_acc);
    spikeIndexMocap = find(accNormMocap > acc_threshold, 1, 'first');   
    
    if isempty(spikeIndexIMU) || isempty(spikeIndexMocap)
        warning(['DECAR_MOCAP_TOOLS: Spike not detected in IMU or Mocap!',...
                'we will assume the data starts at almost the same time.'])
        tSyncIMU = dataIMU.t(1);
        tSyncMocap = t_mocap(1);
    else
        tSyncIMU = dataIMU.t(spikeIndexIMU);  % time of spike in imu clock
        tSyncMocap = t_mocap(spikeIndexMocap); % time of spike in mocap clock.
    end
    
    % the offset, to be used to extract Mocap data for the UWB data as well
    % if offset is positive, this means the mocap detected the spike later
    % (in its clock) than the IMU. This means that the IMU was started
    % afterwards, and thus the mocap is AHEAD of the IMU.
    offset = tSyncMocap - tSyncIMU;
                                              
    %% Generating the synced data
    % Move the IMU clock to the mocap clock.
    if ~force_sync
        t_synced = dataIMU.t + offset; % Shift up IMU time to match mocap time.
    end
    
    % Delete IMU data that doesnt have ground truth.
    isOutsideMocap = (t_synced < 0) | (t_synced > t_mocap(end));
    t_synced = t_synced(~isOutsideMocap);
    imu_acc_synced = dataIMU.accel(:,~isOutsideMocap);
    imu_gyr_synced = dataIMU.gyro(:,~isOutsideMocap);
    imu_mag_synced = dataIMU.mag(:,~isOutsideMocap);
    imu_pressure_synced = dataIMU.pressure(~isOutsideMocap);

    % Refine further using least-squares
    % TODO: make this step optional, as it takes a decent amount of time.
    % TODO: add scaling factor? t_synced_refined = scale*(t_synced + dt);
    if ~force_sync
        f = @(dt) error(dt, t_synced, splineStruct, imu_acc_synced, imu_gyr_synced);
        options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt',...
                               'display','iter-detailed','steptolerance',1e-8,...
                               'InitDamping',0);
        dt = lsqnonlin(f,0,[],[],options);

        % Apply results to the new clock.
        t_synced = t_synced + dt;
        
        % Total offset such that t_synced = data_imu.t + offset;
        offset = offset + dt; 
    end

    % Manual offset if specified by user
    t_synced = t_synced + manual_offset;
    offset = offset + manual_offset;

    [mocap_acc_synced, mocap_omega_synced, dataSynced] = ...
                              getFakeImuMocap(splineStruct, t_synced, g_a);
    
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
    dataSynced.t_0 = dataIMU.t_0;
    dataSynced.t = t_synced;
    dataSynced.accel = imu_acc_synced;
    dataSynced.gyro = imu_gyr_synced;
    dataSynced.mag = imu_mag_synced;
    dataSynced.accel_mocap = mocap_acc_synced;
    dataSynced.gyro_mocap = mocap_omega_synced;
    dataSynced.pressure = imu_pressure_synced;
    dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, splineStruct.gapIntervals);
    dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, splineStruct.staticIntervals);
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