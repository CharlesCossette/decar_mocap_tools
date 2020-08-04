function [C_sm, biasAcc, biasGyr] = calibrateFrames(dataSynced, x, TOL)
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
    biasAcc = x(4:6);
    biasGyr = x(7:9);
    C_sm = ROTVEC_TO_DCM(phi);
    
    f = @(C, bAcc, bGyr) computeErrorVector(C, bAcc, bGyr, dataSynced);

    % Compute the Jacobian w.r.t. the biases
%     A_biases = [-repmat(eye(3),numAcc,1), zeros(3*numAcc,3);...
%                 zeros(3*numGyr,3),        -repmat(eye(3),numGyr,1)];
%                             
    iter = 0;
    while norm(delta) > TOL
        % compute error vector
        e = f(C_sm, biasAcc, biasGyr);
        
        % compute Jacobian
        f_Csm = @(C) f(C, biasAcc, biasGyr);
        A_phi = complexStepJacobianLie(f_Csm,C_sm,3,@CrossOperator,'direction','left');
        
        f_bias = @(b) f(C_sm, b(1:3), b(4:6));
        A_bias_CS = complexStepJacobian(f_bias, [biasAcc;biasGyr]);
        
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
        del_biasAcc = delta(4:6);
        del_biasGyr = delta(7:9);
        
        % Update
        C_sm = ROTVEC_TO_DCM(del_phi).'*C_sm;
        biasAcc = biasAcc + del_biasAcc;
        biasGyr = biasGyr + del_biasGyr;
        
        iter = iter + 1;
    end

    %% Plotting to evaluate performance visually
    accMocap_calibrated   = C_sm * dataSynced.accMocap - biasAcc;
    omegaMocap_calibrated = C_sm * dataSynced.omegaMocap - biasGyr;

    % Plot calibrated data - accelerometers
    figure
    sgtitle('Accelerometers')
    subplot(3,1,1)
    plot(dataSynced.t, accMocap_calibrated(1,:))
    hold on
    plot(dataSynced.t, dataSynced.accIMU(1,:))
    grid
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\ddot{x}$ [$m/s^2$]', 'Interpreter', 'Latex')
    legend('Calibrated Mocap Data', 'IMU Data')
    subplot(3,1,2)
    plot(dataSynced.t, accMocap_calibrated(2,:))
    hold on
    plot(dataSynced.t, dataSynced.accIMU(2,:))
    grid
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\ddot{y}$ [$m/s^2$]', 'Interpreter', 'Latex')
    subplot(3,1,3)
    plot(dataSynced.t, accMocap_calibrated(3,:))
    hold on
    plot(dataSynced.t, dataSynced.accIMU(3,:))
    grid
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\ddot{z}$ [$m/s^2$]', 'Interpreter', 'Latex')

    % Plot calibrated data - gyroscopes
    figure
    sgtitle('Gyroscopes')
    subplot(3,1,1)
    plot(dataSynced.t, omegaMocap_calibrated(1,:))
    hold on
    plot(dataSynced.t, dataSynced.omegaIMU(1,:))
    grid
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\omega_x$ [rad/$s$]', 'Interpreter', 'Latex')
    legend('Calibrated Mocap Data', 'IMU Data')
    subplot(3,1,2)
    plot(dataSynced.t, omegaMocap_calibrated(2,:))
    hold on
    plot(dataSynced.t, dataSynced.omegaIMU(2,:))
    grid
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\omega_y$ [rad/$s$]', 'Interpreter', 'Latex')
    subplot(3,1,3)
    plot(dataSynced.t, omegaMocap_calibrated(3,:))
    hold on
    plot(dataSynced.t, dataSynced.omegaIMU(3,:))
    grid
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\omega_z$ [rad/$s$]', 'Interpreter', 'Latex')

    % Plot evolution of cost function 
    figure
    plot(costFuncHist)
    grid
    xlabel('Iteration Number', 'Interpreter', 'Latex')
    ylabel('$J$ [$\left(m/s^2\right)^2$]', 'Interpreter', 'Latex')
    
end
function output = computeErrorVector(C_sm, bAcc, bGyr, dataSynced)
    gaps = dataSynced.gapIndices;
    error_accel = C_sm*dataSynced.accMocap(:,~gaps) - (dataSynced.accIMU(:,~gaps) + bAcc);
    error_omega = C_sm*dataSynced.omegaMocap(:,~gaps) - (dataSynced.omegaIMU(:,~gaps) + bGyr);
    output = [error_accel(:);error_omega(:)];
end