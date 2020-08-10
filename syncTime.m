function [dataSynced, offset] = syncTime(splineStruct, dataIMU, accThreshold)
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

    g_a = [0;0;-9.80665]; % gravity 
    
    t_mocap = splineStruct.breaks;
    
    %% Generating Mocap accelerometer data using fitted spline
    [mocap_acc, ~] = getFakeImuMocap(splineStruct,t_mocap,g_a);

    %% Finding the timestep in which a spike occurs in both datasets
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
    index_synced       = [];
    t_synced           = [];
    
    % Find the timesteps in the Mocap time reference and its corresponding
    % index in the IMU data
    for lv1=1:length(dataIMU.t)
        tDiff = dataIMU.t(lv1) - tSyncIMU; % the time difference between the 
                                       % the current timestamp and the 
                                       % timestap at which the peak occurred.
        t = tDiff + tSyncMocap; % the corresponding time in the Mocap time reference.
        if t < 0 || t > t_mocap(length(t_mocap))
            continue
        end
        
        index_synced = [index_synced; lv1];
        t_synced = [t_synced; t];
    end

    % Find IMU measurement timestamps to the Mocap clock
    IMU_acc_synced     = zeros(3,length(t_synced));
    IMU_gyr_synced     = zeros(3,length(t_synced));
    for lv1=1:length(t_synced)
        indx = index_synced(lv1);
        IMU_acc_synced(:,lv1) = dataIMU.accel(:,indx);
        IMU_gyr_synced(:,lv1) = dataIMU.gyro(:,indx);
    end
    
    % Emulate accelerometer and gyroscope measurements at required timesteps.
    [mocap_acc_synced, mocap_omega_synced, dataSynced] = getFakeImuMocap(splineStruct, t_synced, g_a);
    
    %% Visualizing the time synchronization
    figure
    plot(t_synced, vecnorm(IMU_acc_synced))
    hold on
    plot(t_synced, vecnorm(mocap_acc_synced))
    hold off
    grid on
    xlabel('$x$ [s]','interpreter','latex')
    ylabel('$a^{zw/a/a}$ [m/s$^2$]','interpreter','latex')
    legend('IMU Data','Mocap Data')
    
    figure
    plot(t_synced, vecnorm(IMU_gyr_synced))
    hold on
    plot(t_synced, vecnorm(mocap_omega_synced))
    hold off   
    xlabel('$t$ [s]','interpreter','latex')
    ylabel('$\omega^{ba}_b$','interpreter','latex')
    grid on
    legend('IMU Data','Mocap Data')
    title('Angular Velocity')
    %% Save the synchronized data
    dataSynced.t          = t_synced;
    dataSynced.accIMU     = IMU_acc_synced;
    dataSynced.omegaIMU   = IMU_gyr_synced;
    dataSynced.accMocap   = mocap_acc_synced;
    dataSynced.omegaMocap = mocap_omega_synced;
    
end
function omega_ba_b = quatrate2omega(q_ba, q_ba_dot)
eta = q_ba(1);
epsilon = q_ba(2:4);
S = [-2*epsilon, 2*(eta*eye(3) - CrossOperator(epsilon))];
omega_ba_b = S*q_ba_dot;
end