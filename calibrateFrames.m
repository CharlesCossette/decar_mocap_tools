function [C_ms, biasAcc_s, biasGyr_s, dataCalibrated] = calibrateFrames(dataSynced, x, TOL)
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
    biasAcc_s = x(4:6);
    biasGyr_s = x(7:9);
    C_ms = ROTVEC_TO_DCM(phi);
    
    f = @(C, bAcc, bGyr) computeErrorVector(C, bAcc, bGyr, dataSynced);

    % Compute the Jacobian w.r.t. the biases
%     A_biases = [-repmat(eye(3),numAcc,1), zeros(3*numAcc,3);...
%                 zeros(3*numGyr,3),        -repmat(eye(3),numGyr,1)];
%                             
    iter = 0;
    while norm(delta) > TOL && iter < 100
        % compute error vector
        e = f(C_ms, biasAcc_s, biasGyr_s);
        
        % compute Jacobian
        f_Cms = @(C) f(C, biasAcc_s, biasGyr_s);
        A_phi = complexStepJacobianLie(f_Cms,C_ms,3,@CrossOperator,'direction','left');
        
        f_bias = @(b) f(C_ms, b(1:3), b(4:6));
        A_bias_CS = complexStepJacobian(f_bias, [biasAcc_s;biasGyr_s]);
        
        A     = [A_phi, A_bias_CS];
        
        cost = 0.5*(e.'*e)
        costFuncHist = [costFuncHist; cost];
        
        % Compute step direction
        if iter > 20
            delta = -(A.'*A + 0.2*diag(diag(A.'*A))) \ (A.' * e);
        else
            delta = -(A.'*A) \ A.' * e;
        end
        
        % decompose
        del_phi     = delta(1:3);
        del_biasAcc_s = delta(4:6);
        del_biasGyr_s = delta(7:9);
        
        % Update
        C_ms = ROTVEC_TO_DCM(del_phi).'*C_ms;
        biasAcc_s = biasAcc_s + del_biasAcc_s;
        biasGyr_s = biasGyr_s + del_biasGyr_s;
        
        iter = iter + 1;
    end
    
    % compute error vector
    e = f(C_ms, biasAcc_s, biasGyr_s);
    RMSE = sqrt((e.'*e)./length(dataSynced.t));
    disp(['RMSE After Calibration: ' , num2str(RMSE)])
    %% Plotting to evaluate performance visually
    accIMU_calibrated = C_ms*(dataSynced.accIMU + biasAcc_s);
    omegaIMU_calibrated = C_ms*(dataSynced.omegaIMU + biasGyr_s);
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
function output = computeErrorVector(C_ms, bAcc, bGyr, dataSynced)
    gaps = dataSynced.gapIndices;
    error_accel = dataSynced.accMocap(:,~gaps) - C_ms*(dataSynced.accIMU(:,~gaps) + bAcc);
    error_omega = dataSynced.omegaMocap(:,~gaps) - C_ms*(dataSynced.omegaIMU(:,~gaps) + bGyr);
    output = [error_accel(:);error_omega(:)];
end