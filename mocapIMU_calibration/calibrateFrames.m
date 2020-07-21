function C_sm = calibrateFrames(syncedData, phi, TOL)
% Find the DCM C_sm representing the rotation between the sensor frame of the 
% IMU (F_s) and the body frame assigned by the Mocap system (F_m).

    if nargin < 1
        error('Missing data')
    end
    if nargin <2
        phi = [0;0;0]; % initial guess
    end
    if nargin < 3
        TOL = 1E-8; % default nonlinear least squares convergence tolerance 
    end
    
    numAcc  = length(syncedData.accIMU);   % number of accel meas
    numGyr  = length(syncedData.omegaIMU); % number of gyr meas
    numMeas = numAcc + numGyr;             % total number of accel 
                                           % and gyro meas.
    delta = Inf;

    %% LS Optimization on SO(3)
    C_sm = ROTVEC_TO_DCM(phi);
    while norm(delta) > TOL
        e = zeros(3*numMeas, 1); % error vector
        A = zeros(3*numMeas, 3); % (\partial C_sm) / (\partial \phi)
        for lv1=1:1:numAcc
            i = 3*(lv1-1)+1;
            j = 3*lv1;
            % compute error vector for acc meas
            e(i:j,:) = C_sm * syncedData.accMocap(:,lv1) - syncedData.accIMU(:,lv1);

            % compute A matrix for acc meas
            A(i:j,:) = -CrossOperator(C_sm * syncedData.accMocap(:,lv1));% * C_sm;
        end
        for lv1=1:1:numGyr
            i = numAcc + 3*(lv1-1)+1;
            j = numAcc + 3*lv1;
            % compute error vector for gyr meas
            e(i:j,:) = C_sm * syncedData.omegaMocap(:,lv1) - syncedData.omegaIMU(:,lv1);

            % compute A matrix for gyr meas
            A(i:j,:) = -CrossOperator(C_sm * syncedData.omegaMocap(:,lv1));% * C_sm;
        end

        % update step
        delta = -(A'*A) \ A' * e;
        C_sm = ROTVEC_TO_DCM(delta)*C_sm;
    end

    %% Plotting to evaluate performance
    accMocap_calibrated = zeros(3,numAcc);
    for lv1=1:1:numAcc
        accMocap_calibrated(1:3,lv1) = C_sm * syncedData.accMocap(:,lv1);
    end
    omegaMocap_calibrated = zeros(3,numGyr);
    for lv1=1:1:numGyr
        omegaMocap_calibrated(1:3,lv1) = C_sm * syncedData.omegaMocap(:,lv1);
    end

    plot(accMocap_calibrated(3,:));
    hold on
    plot(syncedData.accIMU(3,:));

    figure 
    plot(omegaMocap_calibrated(3,:));
    hold on
    plot(syncedData.omegaIMU(3,:));
    
end