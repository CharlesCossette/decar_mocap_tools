function [C_ma, C_mg, biasAcc_a, biasGyr_g, dataCalibrated] = calibrateFrames(dataSynced, x, TOL)
% Find the DCM C_sm representing the rotation between the sensor frame of the 
% IMU (F_s) and the body frame assigned by the Mocap system (F_m), as well
% as the accelerometer and gyroscope biases.
% Requires the complexStepJacobianLie code from decar_utils.

    if nargin < 1
        error('Missing data')
    end
    if nargin <2
        x = [0;0;0;...      % default initial phi
             0;0;0;...      % default initial accelerometer bias
             0;0;0];        % default initial gyroscope bias
    end
    if nargin < 3
        TOL = 1E-8; % default nonlinear least squares convergence tolerance 
    end
    
    numAcc  = length(dataSynced.accIMU);   % number of accel meas
    numGyr  = length(dataSynced.omegaIMU); % number of gyr meas
    delta = Inf;

    %% LS Optimization on SO(3), using Gauss-Newton
    
    % initialize 
    costFuncHist = [];
    phi = x(1:3);
    biasAcc_a = x(4:6);
    biasGyr_g = x(7:9);
    C_ma = ROTVEC_TO_DCM(phi);
    C_mg = ROTVEC_TO_DCM(phi);
                            
    iter = 0;
    while norm(delta) > TOL && iter < 100
        % compute error vector
        e = errorFull(C_ma, C_mg, biasAcc_a, biasGyr_g, dataSynced);
        
        % compute Jacobians
        f_Cma = @(C) errorFull(C, C_mg, biasAcc_a, biasGyr_g, dataSynced);
        A_phia = complexStepJacobianLie(f_Cma,C_ma,3,@CrossOperator,'direction','left');
        
        f_Cmg = @(C) errorFull(C_ma, C, biasAcc_a, biasGyr_g, dataSynced);
        A_phig = complexStepJacobianLie(f_Cmg,C_ma,3,@CrossOperator,'direction','left');
        
        f_bias = @(b) errorFullStaticOnly(C_ma, C_mg, b(1:3), b(4:6), dataSynced);
        %f_bias = @(b) errorFull(C_ma, C_mg, b(1:3), b(4:6), dataSynced);
        A_bias_CS = complexStepJacobian(f_bias, [biasAcc_a;biasGyr_g]);
        
        A     = [A_phia, A_phig, A_bias_CS];
        
        cost = 0.5*(e.'*e)
        costFuncHist = [costFuncHist; cost];
        
        % Compute step direction
        if iter > 1
            delta = -(A.'*A + 0.2*diag(diag(A.'*A))) \ (A.' * e);
        else
            delta = -(A.'*A) \( A.' * e);
        end
        
        % decompose
        del_phia    = delta(1:3);
        del_phig    = delta(4:6);
        del_biasAcc_s = delta(7:9);
        del_biasGyr_s = delta(10:12);
        
        % Update
        C_ma = ROTVEC_TO_DCM(del_phia).'*C_ma;
        C_mg = ROTVEC_TO_DCM(del_phig).'*C_mg;
        biasAcc_a = biasAcc_a + del_biasAcc_s;
        biasGyr_g = biasGyr_g + del_biasGyr_s;
        
        iter = iter + 1;
    end
    
    % compute error vector
    e = errorFull(C_ma, C_mg, biasAcc_a, biasGyr_g, dataSynced);
    RMSE = sqrt((e.'*e)./length(dataSynced.t));
    disp(['RMSE After Calibration: ' , num2str(RMSE)])
    
    %% Plotting to evaluate performance visually
    accIMU_calibrated = C_ma*(dataSynced.accIMU + biasAcc_a);
    omegaIMU_calibrated = C_mg*(dataSynced.omegaIMU + biasGyr_g);
    dataCalibrated = dataSynced;
    dataCalibrated.accIMU = accIMU_calibrated;
    dataCalibrated.omegaIMU = omegaIMU_calibrated;
    
    % Plot calibrated data - accelerometers
    figure
    sgtitle('Accelerometer')
    subplot(3,1,1)
    plot(dataCalibrated.t, dataCalibrated.accIMU(1,:))
    hold on
    plot(dataCalibrated.t, dataCalibrated.accMocap(1,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\ddot{x}$ [$m/s^2$]', 'Interpreter', 'Latex')
    legend('Calibrated IMU Data', 'Mocap Data')
    subplot(3,1,2)
    plot(dataCalibrated.t, dataCalibrated.accIMU(2,:))
    hold on
    plot(dataCalibrated.t, dataCalibrated.accMocap(2,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\ddot{y}$ [$m/s^2$]', 'Interpreter', 'Latex')
    subplot(3,1,3)
    plot(dataCalibrated.t, dataCalibrated.accIMU(3,:))
    hold on
    plot(dataCalibrated.t, dataCalibrated.accMocap(3,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\ddot{z}$ [$m/s^2$]', 'Interpreter', 'Latex')

    % Plot calibrated data - gyroscopes
    figure
    sgtitle('Gyroscope')
    subplot(3,1,1)
    plot(dataCalibrated.t, dataCalibrated.omegaIMU(1,:))
    hold on
    plot(dataCalibrated.t, dataCalibrated.omegaMocap(1,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\omega_x$ [rad/$s$]', 'Interpreter', 'Latex')
    legend('Calibrated IMU Data', 'Mocap Data')
    
    subplot(3,1,2)
    plot(dataCalibrated.t, dataCalibrated.omegaIMU(2,:))
    hold on
    plot(dataCalibrated.t, dataCalibrated.omegaMocap(2,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\omega_y$ [rad/$s$]', 'Interpreter', 'Latex')
    subplot(3,1,3)
    plot(dataCalibrated.t, dataCalibrated.omegaIMU(3,:))
    hold on
    plot(dataCalibrated.t, dataCalibrated.omegaMocap(3,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\omega_z$ [rad/$s$]', 'Interpreter', 'Latex')

    % Plot evolution of cost function 
    figure
    plot(costFuncHist)
    grid
    xlabel('Iteration Number', 'Interpreter', 'Latex')
    ylabel('$J$ [$\left(m/s^2\right)^2$]', 'Interpreter', 'Latex')
    
end
function output = errorFull(C_ma, C_mg, bAcc, bGyr, dataSynced)
    ea = errorAccel(C_ma, bAcc, dataSynced);
    eg = errorGyro(C_mg, bGyr, dataSynced);
    output = [ea;eg];
end
function output = errorAccel(C_ma, bAcc, dataSynced)
    isGap = dataSynced.gapIndices;
    error_accel = dataSynced.accMocap(:,~isGap) ...
                  - C_ma*(dataSynced.accIMU(:,~isGap) + bAcc);
    output = error_accel(:);
end
function output = errorGyro(C_mg, bGyr, dataSynced)
    isGap = dataSynced.gapIndices;
    error_gyro = dataSynced.omegaMocap(:,~isGap) ...
                  - C_mg*(dataSynced.omegaIMU(:,~isGap) + bGyr);
    output = error_gyro(:);
end
function output = errorFullStaticOnly(C_ma, C_mg, bAcc, bGyr, dataSynced)
    ea = errorAccelStaticOnly(C_ma, bAcc, dataSynced);
    eg = errorGyroStaticOnly(C_mg, bGyr, dataSynced);
    output = [ea;eg];
end
function output = errorAccelStaticOnly(C_ma, bAcc, dataSynced)
    isGap = dataSynced.gapIndices;
    isStatic = dataSynced.staticIndices;
    error_accel = dataSynced.accMocap ...
                  - C_ma*(dataSynced.accIMU + bAcc);
    error_accel(:,~isStatic) = 0;
    error_accel = error_accel(:,~isGap);
    output = error_accel(:);
end
function output = errorGyroStaticOnly(C_mg, bGyr, dataSynced)
    isGap = dataSynced.gapIndices;
    isStatic = dataSynced.staticIndices;
    error_gyro = dataSynced.omegaMocap ...
                  - C_mg*(dataSynced.omegaIMU + bGyr);
    error_gyro(:,~isStatic) = 0;
    error_gyro = error_gyro(:,~isGap);
    output = error_gyro(:);
end