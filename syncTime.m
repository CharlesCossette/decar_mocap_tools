function [dataSynced, offset] = syncTime(splineStruct, dataIMU, accThreshold, force_sync)
% Synchronizes the Mocap data represented as a spline and the IMU data,
% based on a spike in acceleration readings.
% The input should be for one rigid body and one IMU.
% Currently outputing only the IMU accelerometer and the gyroscope.
% Requires the bspline code from decar_utils.
% TODO: 1) add mag, baro..

    if nargin < 2
        error('Missing data')
    end
    if nargin < 3
        accThreshold = 12; 
    end
    if ~exist('force_sync','var')
        force_sync = false;
    end

    g_a = [0;0;-9.80665]; % gravity 
    
    t_mocap = splineStruct.breaks;
    
    %% Generating Mocap accelerometer data using fitted spline
    [mocap_acc, ~] = getFakeImuMocap(splineStruct,t_mocap,g_a);

    %% Finding the timestep in which a spike occurs in both datasets
    % TODO: Find a solution when there is no spike....
    % Finding the peak in the IMU data
    accNormIMU    = vecnorm(dataIMU.accel);
    spikeIndexIMU = find(accNormIMU>accThreshold,1,'first');
    tSyncIMU      = dataIMU.t(spikeIndexIMU);  % the timestamp of the spike
                                               % in the IMU data 
                                                   
    % Finding the peak in the Mocap data
    accNormMocap    = vecnorm(mocap_acc);
    spikeIndexMocap = find(accNormMocap>accThreshold,1,'first');
    tSyncMocap      = t_mocap(spikeIndexMocap); % the timestep of the spike in 
                                              % the Mocap ground truth data
    
    % the offset, to be used to extract Mocap data for the UWB data as well
    offset          = tSyncMocap - tSyncIMU;
                                              
    %% Generating the synced data
    % Move the IMU clock to the mocap clock.
    if ~force_sync
        dataIMU.t = dataIMU.t - tSyncIMU + tSyncMocap;
    end
    % Delete IMU data that doesnt have ground truth.
    isOutsideMocap = (dataIMU.t < 0) | (dataIMU.t > t_mocap(end));
    t_synced = dataIMU.t(~isOutsideMocap);
    imu_acc_synced = dataIMU.accel(:,~isOutsideMocap);
    imu_gyr_synced = dataIMU.gyro(:,~isOutsideMocap);
    imu_mag_synced = dataIMU.mag(:,~isOutsideMocap);

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
    end
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
    dataSynced.t = t_synced;
    dataSynced.accel = imu_acc_synced;
    dataSynced.gyro = imu_gyr_synced;
    dataSynced.mag = imu_mag_synced;
    dataSynced.accel_mocap = mocap_acc_synced;
    dataSynced.gyro_mocap = mocap_omega_synced;
    
end

function output = error(dt, t_synced, splineStruct, imu_accel, imu_gyro)
    g_a = [0;0;-9.80665];
    [mocap_acc, mocap_gyro, ~] = getFakeImuMocap(splineStruct, t_synced + dt, g_a);
    error_accel = vecnorm(mocap_acc) - vecnorm(imu_accel);
    error_gyro = vecnorm(mocap_gyro) - vecnorm(imu_gyro);
    output = [0*error_accel(:); error_gyro(:)];
end