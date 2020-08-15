function [results, dataCalibrated] = calibrateFrames(dataSynced, options)
% Find the DCM C_sm representing the rotation between the sensor frame of the
% IMU (F_s) and the body frame assigned by the Mocap system (F_m), as well
% as the accelerometer and gyroscope biases.
% Requires the complexStepJacobianLie code from decar_utils.

% TODO: input parser to turn off/on the different calibration parameters.
if nargin < 1
    error('Missing data')
end
if nargin < 2
    options = struct();
end

if isfield(options,'tolerance')
    TOL = options.tolerance;
else
    TOL = 1e-10;
end
if isfield(options,'frames')
    frames = options.frames;
else
    frames = true;
end

if isfield(options,'bias')
    bias = options.bias;
else
    bias = true;
end

if isfield(options,'scale')
    scale = options.scale;
else
    scale = true;
end

if isfield(options,'skew')
    skew = options.skew;
else
    skew = false;
end

if isfield(options,'grav')
    grav = options.grav;
else
    grav = false;
end

%% LS Optimization on SO(3), using Gauss-Newton

% Extract covariances when static as weighting matrices.
if sum(dataSynced.staticIndices) > 3
    cov_a = cov(dataSynced.accMocap(:,dataSynced.staticIndices).');
    cov_g = cov(dataSynced.omegaMocap(:,dataSynced.staticIndices).');
    W_a = inv(diag(diag(chol(cov_a))));
    W_g = inv(diag(diag(chol(cov_g))));
else
    W_a = eye(3);
    W_g = eye(3);
end
% initialize
costFuncHist = [];
phi = [0;0;0];
bias_a = [0;0;0];
bias_g = [0;0;0];
C_ma = ROTVEC_TO_DCM(phi);
C_mg = ROTVEC_TO_DCM(phi);
scale_a = [1;1;1];
scale_g = [1;1;1];
skew_a = [0;0;0];
skew_g = [0;0;0];
C_ae = eye(3);

delta = Inf;
iter = 0;
while norm(delta) > TOL && iter < 100
    indxCounter = 1;
    % compute error vector
    e = errorFull(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, W_a, W_g);
    
    % compute Jacobians
    A = [];
    
    if frames
        f_C_ma = @(C) errorFull(C, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, W_a, W_g);
        A_phi_a = complexStepJacobianLie(f_C_ma,C_ma,3,@CrossOperator,'direction','left');

        f_C_mg = @(C) errorFull(C_ma, C, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, W_a, W_g);
        A_phi_g = complexStepJacobianLie(f_C_mg,C_mg,3,@CrossOperator,'direction','left');

        A = [A, A_phi_a, A_phi_g];
        phiIndices = indxCounter:indxCounter+5;
        indxCounter = indxCounter + 6;
    end
    
    if bias
        f_bias = @(b) errorFull(C_ma, C_mg, b(1:3), b(4:6), scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, W_a, W_g);
        A_bias = complexStepJacobian(f_bias, [bias_a;bias_g]);
        A = [A, A_bias];
        bias_indices = indxCounter:indxCounter + 5;
        indxCounter = indxCounter + 6;
    end
    
    if scale
        f_scale = @(s) errorFull(C_ma, C_mg, bias_a, bias_g, s(1:3), s(4:6), skew_a, skew_g, C_ae, dataSynced, W_a, W_g);
        A_scale = complexStepJacobian(f_scale, [scale_a;scale_g]);
        A = [A,A_scale];
        scaleIndices = indxCounter:indxCounter + 5;
        indxCounter = indxCounter + 6;
    end
    
    if skew
        f_skew = @(s) errorFull(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, s(1:3), s(4:6), C_ae, dataSynced, W_a, W_g);
        A_skew = complexStepJacobian(f_skew, [skew_a;skew_g]);
        A = [A, A_skew];
        skewIndices = indxCounter:indxCounter + 5;
        indxCounter = indxCounter + 6;
    end
    
    if grav
        f_Cae = @(C) errorFull(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C, dataSynced, W_a, W_g);
        f_phiae = @(phi) f_Cae(expmTaylor(CrossOperator([phi(1);phi(2);0])*C_ae));
        A_phiae = complexStepJacobian(f_phiae,[0;0]);
        A = [A, A_phiae];
        gravIndices = indxCounter:indxCounter + 1;
    end
    
    cost = 0.5*(e.'*e)
    costFuncHist = [costFuncHist; cost];
    
    if isempty(A)
        warning('No calibration parameters have been selected.')
        delta = 1e-16
    else
        % Compute step direction
        if iter > 20
            delta = -(A.'*A + 0.2*diag(diag(A.'*A))) \ (A.' * e);
        else
            delta = -(A.'*A) \( A.' * e);
        end
    end
    
    % decompose and update
    if frames
        del_phia    = delta(phiIndices(1:3));
        del_phig    = delta(phiIndices(4:6));
        C_ma = ROTVEC_TO_DCM(-del_phia)*C_ma;
        C_mg = ROTVEC_TO_DCM(-del_phig)*C_mg;
    end
    
    if bias
        del_biasAcc_s = delta(bias_indices(1:3));
        del_biasGyr_s = delta(bias_indices(4:6));
        bias_a = bias_a + del_biasAcc_s;
        bias_g = bias_g + del_biasGyr_s;
    end
    
    if scale
        del_scale_a = delta(scaleIndices(1:3));
        del_scale_g = delta(scaleIndices(4:6));
        scale_a = scale_a + del_scale_a;
        scale_g = scale_g + del_scale_g;
    end
    
    if skew
        del_skew_a = delta(skewIndices(1:3));
        del_skew_g = delta(skewIndices(4:6));
        skew_a = skew_a + del_skew_a;
        skew_g = skew_g + del_skew_g;
    end
    
    if grav
        del_phiae = delta(gravIndices);
        C_ae = ROTVEC_TO_DCM(-[del_phiae;0])*C_ae;
    end

    iter = iter + 1;
end

% compute error vector
e = errorFullNoWeight(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced);
RMSE = sqrt((e.'*e)./length(dataSynced.t));
disp(['RMSE After Calibration: ' , num2str(RMSE)])

%% Results and Output
% Calibrated Accelerometer measurements
T_skew = eye(3);
T_skew(1,2) = -skew_a(3);
T_skew(1,3) =  skew_a(2);
T_skew(2,3) = -skew_a(1);
accIMU_calibrated = C_ma*T_skew*diag(scale_a)*(dataSynced.accIMU + bias_a);

% Calibrated Gyroscope measurements
T_skew = eye(3);
T_skew(1,2) = -skew_g(3);
T_skew(1,3) =  skew_g(2);
T_skew(2,3) = -skew_g(1);
omegaIMU_calibrated = C_mg*T_skew*diag(scale_g)*(dataSynced.omegaIMU + bias_g);

% Calibrated ground truth accel/gyro measurements.
g_e = [0;0;-9.80665];
g_a = C_ae*g_e;
mocap_gyro = dataSynced.omegaMocap;
mocap_accel = zeros(3, length(dataSynced.t));
for lv1 = 1:length(dataSynced.t)
    mocap_accel(:,lv1) = dataSynced.accMocap(:,lv1) - dataSynced.C_ba(:,:,lv1)*(g_a - g_e);
end

dataCalibrated.t = dataSynced.t;
dataCalibrated.accMocap = mocap_accel;
dataCalibrated.omegaMocap = mocap_gyro;
dataCalibrated.accIMU = accIMU_calibrated;
dataCalibrated.omegaIMU = omegaIMU_calibrated;

results.C_ms_accel = C_ma;
results.C_ms_gyro = C_mg;
results.bias_accel = bias_a;
results.bias_gyro = bias_g;
results.scale_accel = scale_a;
results.scale_gyro = scale_g;
results.skew_accel = skew_a;
results.skew_gyro = skew_g;
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
function output = errorFull(C_ma, C_mg, bAcc, bGyr, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, L_a, L_g)
    isGap = dataSynced.gapIndices;
    isStatic = dataSynced.staticIndices;
    ea = errorAccel(C_ma, bAcc, scale_a, skew_a, C_ae, dataSynced);
    eg = errorGyro(C_mg, bGyr, scale_g, skew_g, dataSynced);
    ea(:,isStatic) = 50*ea(:,isStatic);
    eg(:,isStatic) = 100*eg(:,isStatic);
    ea = L_a*ea(:,~isGap);
    eg = L_g*eg(:,~isGap);
    output = [ea(:);eg(:)];
end
function output = errorFullNoWeight(C_ma, C_mg, bAcc, bGyr, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced)
    isGap = dataSynced.gapIndices;
    ea = errorAccel(C_ma, bAcc, scale_a, skew_a, C_ae, dataSynced);
    eg = errorGyro(C_mg, bGyr, scale_g, skew_g, dataSynced);
    ea = ea(:,~isGap);
    eg = eg(:,~isGap);
    output = [ea(:);eg(:)];
end
function output = errorAccel(C_ma, bAcc, scale_a, skew_a, C_ae, dataSynced)
    
    if all(C_ae == eye(3))
        mocap_accel = dataSynced.accMocap;
    else
        g_e = [0;0;-9.80665];
        g_a = C_ae*g_e;
        mocap_accel = zeros(3, length(dataSynced.t));
        for lv1 = 1:length(dataSynced.t)
            mocap_accel(:,lv1) = dataSynced.C_ba(:,:,lv1)*(dataSynced.a_zwa_a(:,lv1) - g_a);
        end
    end
    T_skew = eye(3);
    T_skew(1,2) = -skew_a(3);
    T_skew(1,3) =  skew_a(2);
    T_skew(2,3) = -skew_a(1);
    error_accel = mocap_accel...
                  - C_ma*T_skew*diag(scale_a)*(dataSynced.accIMU + bAcc);
    output = error_accel;
end
function output = errorGyro(C_mg, bGyr, scale_g, skew_g, dataSynced)
    T_skew = eye(3);
    T_skew(1,2) = -skew_g(3);
    T_skew(1,3) =  skew_g(2);
    T_skew(2,3) = -skew_g(1);
    error_gyro = dataSynced.omegaMocap ...
                  - C_mg*T_skew*diag(scale_g)*(dataSynced.omegaIMU+ bGyr);
    output = error_gyro;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = errorFullStaticOnly(C_ma, C_mg, bAcc, bGyr, scale_a, scale_g, C_ae, dataSynced)
    isGap = dataSynced.gapIndices;
    isStatic = dataSynced.staticIndices;
    ea = errorAccel(C_ma, bAcc, scale_a, skew_a, C_ae, dataSynced);
    eg = errorGyro(C_mg, bGyr, scale_g, skew_g, dataSynced);
    ea(:,isStatic) = 0;
    eg(:,isStatic) = 0;
    ea = ea(:,~isGap);
    eg = eg(:,~isGap);
    output = [ea(:);eg(:)];
end
