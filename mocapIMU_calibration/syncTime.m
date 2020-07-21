function syncedData = syncTime(bSplineStruct, dataIMU, accThreshold)
% Synchronizes the Mocap data represented as a spline and the IMU data,
% based on a spike in acceleration readings.
% The input should be for one rigid body and one IMU.
% Currently outputing only the IMU accelerometer and the gyroscope.
% TODO: 1) Preallocate memory for computational efficiency
%       2) add mag, baro..
%       3) add user-defined bias corrections (PRIORITY!)

    if nargin < 2
        error('Missing data')
    end
    if nargin < 3
        accThreshold = 12; 
    end

    knots = bSplineStruct.knots;
    P     = bSplineStruct.P;
    
    %% Generating Mocap accelerometer data using fitted spline
    numKnots = length(bSplineStruct.knots);
    mocap_acc = zeros(3, numKnots);
    parfor lv1=1:numKnots
        t = knots(lv1);
        
        % extract attitude
        temp   = bspline(t,knots,P,3);
        phi_ba = temp(4:6);
        C_ba   = ROTVEC_TO_DCM(phi_ba);
        
        % extract acceleration + gravity
        temp = bsplineDerv(t,knots,P,3,2);
        mocap_acc(:,lv1) = temp(1:3) + C_ba*[0.21;-0.6728;9.4]; %% TODO: add user-defined bias corrections
    end
    
    %% Finding the timestep in which a spike occurs in both datasets
    % Finding the peak in the IMU data
    accNormIMU    = vecnorm(dataIMU.accel);
    spikeIndexIMU = find(accNormIMU>accThreshold,1,'first');
    tSyncIMU      = dataIMU.t(spikeIndexIMU);  % the timestamp of the spike
                                               % in the IMU data 
                                                   
    % Finding the peak in the Mocap data
    accNormMocap    = vecnorm(mocap_acc);
    spikeIndexMocap = find(accNormMocap>accThreshold,1,'first');
    tSyncMocap      = knots(spikeIndexMocap); % the timestep of the spike in 
                                              % the Mocap ground truth data
                                              
    %% Generating the synced data
    t_synced           = [];
    IMU_acc_synced     = [];
    IMU_gyr_synced     = [];
    Mocap_acc_synced   = [];
    Mocap_omega_synced = [];
    parfor lv1=1:length(dataIMU.t)
        tDiff = dataIMU.t(lv1) - tSyncIMU; % the time difference between the 
                                       % the current timestamp and the 
                                       % timestap at which the peak occurred.
        t = tDiff + tSyncMocap; % the corresponding time in the Mocap time reference.
        if t < 0 || t > knots(length(knots))
            continue
        end

        % IMU Data
        t_synced = [t_synced, t];
        IMU_acc_synced = [IMU_acc_synced, dataIMU.accel(:,lv1)];
        IMU_gyr_synced = [IMU_gyr_synced, dataIMU.gyro(:,lv1)];

        % Mocap omega data
        temp       = bspline(t,knots,P,3);
        phi_ba     = temp(4:6);
        temp       = bsplineDerv(t,knots,P,3,1);
        phi_ba_dot = temp(4:6);
        temp = PhiDotToOmega(phi_ba, phi_ba_dot);
        Mocap_omega_synced = [Mocap_omega_synced, temp];

        % Mocap acceleration data
        temp = bsplineDerv(t,knots,P,3,2);
        C_ba = ROTVEC_TO_DCM(phi_ba);
        Mocap_acc_synced = [Mocap_acc_synced, temp(1:3)+C_ba*[0.21;-0.6728;9.4]];
    end
    
    
    %% Visualizing the time synchronization
    plot(t_synced, vecnorm(IMU_acc_synced))
    hold on
    plot(t_synced, vecnorm(Mocap_acc_synced))

    figure
    plot(t_synced, vecnorm(IMU_gyr_synced))
    hold on
    plot(t_synced, vecnorm(Mocap_omega_synced))
    
    
    %% Save the synchronized data
    syncedData.t          = t_synced;
    syncedData.accIMU     = IMU_acc_synced;
    syncedData.omegaIMU   = IMU_gyr_synced;
    syncedData.accMocap   = Mocap_acc_synced;
    syncedData.omegaMocap = Mocap_omega_synced;
    
end
function omega_ba_b = PhiDotToOmega(phi_vec, phi_vec_dot)
    phi = norm(phi_vec);
    a = phi_vec/phi;

    if phi < 1e-13
        % No rotation
        omega_ba_b = phi_vec_dot;
    elseif norm(phi_vec_dot) < 1e-13
        % No rate of change
        omega_ba_b = zeros(3,1);
    else
        % Omega to [a_dot;phi_dot] mapping matrix (Gamma matrix)
        G = [0.5*(CrossOperator(a) - cot(phi/2)*CrossOperator(a)*CrossOperator(a)); a.'];
        % [a_dot;phi_dot] to [phi_vec_dot;0] mapping matrix
        A = [[phi*eye(3),a];[2*a.',0]];

        aphidot = A\[phi_vec_dot;0];
        omega_ba_b = G\aphidot;
    end
end