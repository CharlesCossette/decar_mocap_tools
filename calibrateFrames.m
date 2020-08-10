function [results, dataCalibrated] = calibrateFrames(dataSynced, splineStruct, x, TOL)
% Find the DCM C_sm representing the rotation between the sensor frame of the 
% IMU (F_s) and the body frame assigned by the Mocap system (F_m), as well
% as the accelerometer and gyroscope biases.
% Requires the complexStepJacobianLie code from decar_utils.

    if nargin < 1
        error('Missing data')
    end
    if nargin <3
        x = [0;0;0;...      % default initial phi
             0;0;0;...      % default initial accelerometer bias
             0;0;0];        % default initial gyroscope bias
    end
    if nargin < 4
        TOL = 1E-8; % default nonlinear least squares convergence tolerance 
    end
    
    numAcc  = length(dataSynced.accIMU);   % number of accel meas
    numGyr  = length(dataSynced.omegaIMU); % number of gyr meas
    delta = Inf;

    %% LS Optimization on SO(3), using Gauss-Newton
    
    % initialize 
    costFuncHist = [];
    phi = x(1:3);
    bias_a = x(4:6);
    bias_g = x(7:9);
    C_ma = ROTVEC_TO_DCM(phi);
    C_mg = ROTVEC_TO_DCM(phi);
    scale_a = [1;1;1];
    scale_g = [1;1;1];
    skew_a = [0;0;0];
    skew_g = [0;0;0];
    C_ae = eye(3);
                            
    iter = 0;
    while norm(delta) > TOL && iter < 100
        % compute error vector
        e = errorFull(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced);
        
        % compute Jacobians
        f_Cma = @(C) errorFull(C, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced);
        A_phia = complexStepJacobianLie(f_Cma,C_ma,3,@CrossOperator,'direction','left');
        
        f_Cmg = @(C) errorFull(C_ma, C, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced);
        A_phig = complexStepJacobianLie(f_Cmg,C_ma,3,@CrossOperator,'direction','left');
        
        %f_bias = @(b) errorFullStaticOnly(C_ma, C_mg, b(1:3), b(4:6), scale_a, scale_g, dataSynced);
        f_bias = @(b) errorFull(C_ma, C_mg, b(1:3), b(4:6), scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced);
        A_bias = complexStepJacobian(f_bias, [bias_a;bias_g]);
        
        f_scale = @(s) errorFull(C_ma, C_mg, bias_a, bias_g, s(1:3), s(4:6), skew_a, skew_g, C_ae, dataSynced);
        A_scale = complexStepJacobian(f_scale, [bias_a;bias_g]);
        
        f_skew = @(s) errorFull(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, s(1:3), s(4:6), C_ae, dataSynced);
        A_skew = complexStepJacobian(f_skew, [skew_a;skew_g]);
        
        f_Cae = @(C) errorFull(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C, dataSynced);
        f_phiae = @(phi) f_Cae(expmTaylor(CrossOperator([phi(1);phi(2);0])*C_ae));
        A_phiae = complexStepJacobian(f_phiae,[0;0]);
        
        A     = [A_phia, A_phig, A_bias, A_scale, A_skew, A_phiae];
        
        cost = 0.5*(e.'*e)
        costFuncHist = [costFuncHist; cost];
        
        % Compute step direction
        if iter > 20
            delta = -(A.'*A + 0.2*diag(diag(A.'*A))) \ (A.' * e);
        else
            delta = -(A.'*A) \( A.' * e);
        end
        
        % decompose
        del_phia    = delta(1:3);
        del_phig    = delta(4:6);
        del_biasAcc_s = delta(7:9);
        del_biasGyr_s = delta(10:12);
        del_scale_a = delta(13:15);
        del_scale_g = delta(16:18);
        del_skew_a = delta(19:21);
        del_skew_g = delta(22:24);
        del_phiae = delta(25:26);
        
        % Update
        C_ma = ROTVEC_TO_DCM(del_phia).'*C_ma;
        C_mg = ROTVEC_TO_DCM(del_phig).'*C_mg;
        bias_a = bias_a + del_biasAcc_s;
        bias_g = bias_g + del_biasGyr_s;
        scale_a = scale_a + del_scale_a;
        scale_g = scale_g + del_scale_g;
        skew_a = skew_a + del_skew_a;
        skew_g = skew_g + del_skew_g;
        C_ae = ROTVEC_TO_DCM([del_phiae;0]).'*C_ae;
        
        iter = iter + 1;
    end
    
    % compute error vector
    e = errorFull(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced);
    RMSE = sqrt((e.'*e)./length(dataSynced.t));
    disp(['RMSE After Calibration: ' , num2str(RMSE)])
    
    %% Results and Output
    % Calibrated Accelerometer measurements
    T_skew = eye(3);
    T_skew(1,2) = -skew_a(3);
    T_skew(1,3) = skew_a(2);
    T_skew(2,3) = -skew_a(1);
    accIMU_calibrated = C_ma*T_skew*diag(scale_a)*(dataSynced.accIMU + bias_a);
    
    % Calibrated Gyroscope measurements
    T_skew = eye(3);
    T_skew(1,2) = -skew_g(3);
    T_skew(1,3) = skew_g(2);
    T_skew(2,3) = -skew_g(1);
    omegaIMU_calibrated = C_mg*T_skew*diag(scale_g)*(dataSynced.omegaIMU + bias_g);
    
    % Calibrated ground truth accel/gyro measurements.
    g_e = [0;0;-9.80665];
    [mocap_acc, mocap_gyro] = getFakeImuMocap(splineStruct,dataSynced.t, C_ae*g_e);
    
    dataCalibrated.t = dataSynced.t;
    dataCalibrated.accMocap = mocap_acc;
    dataCalibrated.omegaMocap = mocap_gyro;
    dataCalibrated.accIMU = accIMU_calibrated;
    dataCalibrated.omegaIMU = omegaIMU_calibrated;
    
    results.C_ms_accel = C_ma;
    results.C_ms_gyro = C_mg;
    results.bias_s_accel = bias_a;
    results.bias_s_gyro = bias_g;
    results.scale_s_accel = scale_a;
    results.scale_s_gyro = scale_g;
    results.skew_s_accel = skew_a;
    results.skew_s_gyro = skew_g;
    results.g_a = C_ae*g_e;
    
    %% Plotting to evaluate performance visually
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = errorFull(C_ma, C_mg, bAcc, bGyr, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced)
    ea = errorAccel(C_ma, bAcc, scale_a, skew_a, C_ae, dataSynced);
    eg = errorGyro(C_mg, bGyr, scale_g, skew_g, dataSynced);
    output = [ea;eg];
end
function output = errorAccel(C_ma, bAcc, scale_a, skew_a, C_ae, dataSynced)
    g_e = [0;0;-9.80665];
    g_a = C_ae*g_e;
    mocap_accel = zeros(3, length(dataSynced.t));
    for lv1 = 1:length(dataSynced.t)
        mocap_accel(:,lv1) = dataSynced.C_ba(:,:,lv1)*(dataSynced.a_zwa_a(:,lv1) - g_a);
    end
    isGap = dataSynced.gapIndices;
    T_skew = eye(3);
    T_skew(1,2) = -skew_a(3);
    T_skew(1,3) = skew_a(2);
    T_skew(2,3) = -skew_a(1);
    error_accel = mocap_accel(:,~isGap) ...
                  - C_ma*T_skew*diag(scale_a)*(dataSynced.accIMU(:,~isGap) + bAcc);
    output = error_accel(:);
end
function output = errorGyro(C_mg, bGyr, scale_g, skew_g, dataSynced)
    isGap = dataSynced.gapIndices;
    T_skew = eye(3);
    T_skew(1,2) = -skew_g(3);
    T_skew(1,3) = skew_g(2);
    T_skew(2,3) = -skew_g(1);
    error_gyro = dataSynced.omegaMocap(:,~isGap) ...
                  - C_mg*T_skew*diag(scale_g)*(dataSynced.omegaIMU(:,~isGap) + bGyr);
    output = error_gyro(:);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = errorFullStaticOnly(C_ma, C_mg, bAcc, bGyr, scale_a, scale_g, C_ae, dataSynced, splineStruct)
    ea = errorAccelStaticOnly(C_ma, bAcc, scale_a, dataSynced);
    eg = errorGyroStaticOnly(C_mg, bGyr, scale_g, dataSynced);
    output = [ea;eg];
end
function output = errorAccelStaticOnly(C_ma, bAcc, scale_a, dataSynced)
    isGap = dataSynced.gapIndices;
    isStatic = dataSynced.staticIndices;
    error_accel = dataSynced.accMocap ...
                  - C_ma*diag(scale_a)*(dataSynced.accIMU + bAcc);
    error_accel(:,~isStatic) = 0;
    error_accel = error_accel(:,~isGap);
    output = error_accel(:);
end
function output = errorGyroStaticOnly(C_mg, bGyr, scale_g, dataSynced)
    isGap = dataSynced.gapIndices;
    isStatic = dataSynced.staticIndices;
    error_gyro = dataSynced.omegaMocap ...
                  - C_mg*diag(scale_g)*(dataSynced.omegaIMU + bGyr);
    error_gyro(:,~isStatic) = 0;
    error_gyro = error_gyro(:,~isGap);
    output = error_gyro(:);
end