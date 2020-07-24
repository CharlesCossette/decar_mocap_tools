function syncedData = syncTime(bSplineStruct, dataIMU, accThreshold)
% Synchronizes the Mocap data represented as a spline and the IMU data,
% based on a spike in acceleration readings.
% The input should be for one rigid body and one IMU.
% Currently outputing only the IMU accelerometer and the gyroscope.
% Requires the bspline code from decar_utils.
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
    temp     = bspline(knots,knots,P,3);
    tempDerv = bsplineDerv(knots,knots,P,3,2);
    for lv1=1:numKnots     
        % extract attitude
        q_ba = temp(4:7,lv1);
        C_ba   = quat2dcm(q_ba.');
        
        % extract acceleration + gravity
        mocap_acc(:,lv1) = tempDerv(1:3,lv1) + C_ba*[-0.24;-0.21;9.78]; %% TODO: add user-defined bias corrections
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
    index_synced       = [];
    t_synced           = [];
    
    % Find the timesteps in the Mocap time reference and its corresponding
    % index in the IMU data
    for lv1=1:length(dataIMU.t)
        tDiff = dataIMU.t(lv1) - tSyncIMU; % the time difference between the 
                                       % the current timestamp and the 
                                       % timestap at which the peak occurred.
        t = tDiff + tSyncMocap; % the corresponding time in the Mocap time reference.
        if t < 0 || t > knots(length(knots))
            continue
        end
        
        index_synced = [index_synced; lv1];
        t_synced = [t_synced; t];
    end
    
    % Extract information from the b-spline fit at the required timesteps.
    temp      = bspline(t_synced,knots,P,3);
    tempDerv  = bsplineDerv(t_synced,knots,P,3,1);
    tempDerv2 = bsplineDerv(t_synced,knots,P,3,2);
    
    IMU_acc_synced     = zeros(3,length(t_synced));
    IMU_gyr_synced     = zeros(3,length(t_synced));
    Mocap_acc_synced   = zeros(3,length(t_synced));
    Mocap_omega_synced = zeros(3,length(t_synced));
    for lv1=1:length(t_synced)
        i = index_synced(lv1);
        IMU_acc_synced(:,lv1) = dataIMU.accel(:,i);
        IMU_gyr_synced(:,lv1) = dataIMU.gyro(:,i);

        % Mocap omega data
        q_ba     = temp(4:7,lv1);
        q_ba_dot = tempDerv(4:7,lv1);
        omega = quatrate2omega(q_ba, q_ba_dot);
        Mocap_omega_synced(:,lv1) = omega;

        % Mocap acceleration data
        C_ba = quat2dcm(q_ba.');
        Mocap_acc_synced(:,lv1) = tempDerv2(1:3,lv1)+C_ba*[-0.24;-0.21;9.78];
    end
    
    
    %% Visualizing the time synchronization
    figure
    plot(t_synced, vecnorm(IMU_acc_synced))
    hold on
    plot(t_synced, vecnorm(Mocap_acc_synced))
    hold off
    grid on
    xlabel('$x$ [s]','interpreter','latex')
    ylabel('$a^{zw/a/a}_b$ [m/s^2]','interpreter','latex')
    legend('IMU Data','Mocap Data')
    
    figure
    plot(t_synced, vecnorm(IMU_gyr_synced))
    hold on
    plot(t_synced, vecnorm(Mocap_omega_synced))
    hold off   
    xlabel('$x$ [s]','interpreter','latex')
    ylabel('$\omega^{ba}_b$','interpreter','latex')
    grid on
    legend('IMU Data','Mocap Data')
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
function omega_ba_b = quatrate2omega(q_ba, q_ba_dot)
eta = q_ba(1);
epsilon = q_ba(2:4);
S = [-2*epsilon, 2*(eta*eye(3) - CrossOperator(epsilon))];
omega_ba_b = S*q_ba_dot;
end