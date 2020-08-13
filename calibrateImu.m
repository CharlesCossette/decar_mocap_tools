function [results, dataCalibrated] = calibrateImu(dataSynced, options)
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
    TOL = 1e-6;
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
    scale = false;
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

% Provide an initial guess for the biases.
if any(dataSynced.staticIndices)
    isStatic = dataSynced.staticIndices;
    bias_a = mean(dataSynced.accMocap(:,isStatic) - dataSynced.accIMU(:,isStatic),2);
    bias_g = mean(dataSynced.omegaMocap(:,isStatic) - dataSynced.omegaIMU(:,isStatic),2);
else
    bias_a = [0;0;0];
    bias_g = [0;0;0];
end

% initialize
costFuncHist = [];
phi = [0;0;0];
C_ma = ROTVEC_TO_DCM(phi);
C_mg = ROTVEC_TO_DCM(phi);
scale_a = [1;1;1];
scale_g = [1;1;1];
skew_a = [0;0;0];
skew_g = [0;0;0];
C_ae = eye(3);

params.window_size = length(dataSynced.t);
params.min_index = 1; 
params.interval_size = round(length(dataSynced.t)/20);
params.batch_size = 250;
params.max_index = params.min_index + params.window_size - 1;
if params.max_index > length(dataSynced.t)
    params.max_index = length(dataSynced.t);
end


% compute error vector once
[~, e_pos, e_att] = errorDeadReckoning(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
figure(2)
subplot(3,1,1)
h1 = plot(real(e_pos(1,:)));
axis([-inf inf -1 1])
title('Position Dead-Reckoning Error')
grid on
subplot(3,1,2)
h2 = plot(real(e_pos(2,:)));
axis([-inf inf -1 1])
grid on
subplot(3,1,3)
h3 = plot(real(e_pos(3,:)));
axis([-inf inf -1 1])
grid on

figure(3)
subplot(4,1,1)
h4 = plot(real(e_att(1,:)));
axis([-inf inf -pi pi])
grid on
title('Quaternion Dead-Reckoning Error')
subplot(4,1,2)
h5 = plot(real(e_att(2,:)));
axis([-inf inf -pi pi])
grid on
subplot(4,1,3)
h6 = plot(real(e_att(3,:)));
axis([-inf inf -pi pi])
grid on
subplot(4,1,4)
%h7 = plot(real(e_att(3,:)));
axis([-inf inf -pi pi])
grid on
pause(eps)

delta = Inf;
iter = 0;
delta_cost = Inf;
while norm(delta) > TOL && iter < 100 && delta_cost > 1e-2
    indx_counter = 1;
    
    % compute error vector
    [e, e_pos, e_att] = errorDeadReckoning(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
        
    cost = 0.5*(e.'*e)
    
    if ~isempty(costFuncHist)
        delta_cost = abs((cost - costFuncHist(end))/(cost - costFuncHist(1)));
    end
    costFuncHist = [costFuncHist; cost];
    
    
    h1.YData = e_pos(1,:);
    h2.YData = e_pos(2,:);
    h3.YData = e_pos(3,:);
    h4.YData = e_att(1,:);
    h5.YData = e_att(2,:);
    h6.YData = e_att(3,:);
   % h7.YData = e_att(4,:);
    pause(0.0001)
    
    % compute Jacobians
    A = [];
    
    if frames
        f_Cma = @(C) errorDeadReckoning(C, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
        A_phia = complexStepJacobianLie(f_Cma,C_ma,3,@CrossOperator,'direction','left');

        f_Cmg = @(C) errorDeadReckoning(C_ma, C, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
        A_phig = complexStepJacobianLie(f_Cmg,C_ma,3,@CrossOperator,'direction','left');

        A = [A,A_phia, A_phig];
        phi_indices = indx_counter:indx_counter+5;
        indx_counter = indx_counter + 6;
    end
    
    if bias
        f_bias = @(b) errorDeadReckoning(C_ma, C_mg, b(1:3), b(4:6), scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
        A_bias = complexStepJacobian(f_bias, [bias_a;bias_g]);
        A = [A,A_bias];
        bias_indices = indx_counter:indx_counter + 5;
        indx_counter = indx_counter + 6;
    end
    
    if scale
        f_scale = @(s) errorDeadReckoning(C_ma, C_mg, bias_a, bias_g, s(1:3), s(4:6), skew_a, skew_g, C_ae, dataSynced, params);
        A_scale = complexStepJacobian(f_scale, [bias_a;bias_g]);
        A = [A,A_scale];
        scale_indices = indx_counter:indx_counter + 5;
        indx_counter = indx_counter + 6;
    end
    
    if skew
        f_skew = @(s) errorDeadReckoning(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, s(1:3), s(4:6), C_ae, dataSynced, params);
        A_skew = complexStepJacobian(f_skew, [skew_a;skew_g]);
        A = [A, A_skew];
        skew_indices = indx_counter:indx_counter + 5;
        indx_counter = indx_counter + 6;
    end
    
    if grav
        f_Cae = @(C) errorDeadReckoning(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C, dataSynced, params);
        f_phiae = @(phi) f_Cae(expmTaylor(CrossOperator([phi(1);phi(2);0])*C_ae));
        A_phiae = complexStepJacobian(f_phiae,[0;0]);
        A = [A, A_phiae];
        grav_indices = indx_counter:indx_counter + 1;
    end

    
    if isempty(A)
        warning('No calibration parameters have been selected.')
        delta = 1e-16
    else
        % Compute step direction
        if iter > 5
            delta = -(A.'*A + 0.5*diag(diag(A.'*A))) \ (A.' * e);
        else
            delta = -(A.'*A) \( A.' * e);
        end
    end 
    
    % decompose and update
    if frames
        del_phia = delta(phi_indices(1:3));
        del_phig = delta(phi_indices(4:6));
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
        del_scale_a = delta(scale_indices(1:3));
        del_scale_g = delta(scale_indices(4:6));
        scale_a = scale_a + del_scale_a;
        scale_g = scale_g + del_scale_g;
    end
    
    if skew
        del_skew_a = delta(skew_indices(1:3));
        del_skew_g = delta(skew_indices(4:6));
        skew_a = skew_a + del_skew_a;
        skew_g = skew_g + del_skew_g;
    end
    
    if grav
        del_phiae = delta(grav_indices);
        C_ae = ROTVEC_TO_DCM(-[del_phiae;0])*C_ae;
    end

    iter = iter + 1;
end

% compute error vector
e = errorDeadReckoning(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
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
function [error, error_position, error_attitude] = errorDeadReckoning(...
    C_ma, C_mg, bAcc, bGyr, scale_a, scale_g, skew_a, skew_g, C_ae,...
    dataSynced, params ...
    ) 
% ERRORDEADRECKONING 
% TODO: currently subtracting quaternions, and I hate myself.
% but it seems to work.
    isGap = dataSynced.gapIndices;
    isStatic = dataSynced.staticIndices;   
    interval_size = params.interval_size;
    batch_size = params.batch_size;
    min_index = params.min_index;
    max_index = params.max_index;
    
    T_skew_a = eye(3);
    T_skew_a(1,2) = -skew_a(3);
    T_skew_a(1,3) =  skew_a(2);
    T_skew_a(2,3) = -skew_a(1);
    accel_corrected = C_ma*T_skew_a*diag(scale_a)*(dataSynced.accIMU + bAcc);
    
    T_skew_g = eye(3);
    T_skew_g(1,2) = -skew_g(3);
    T_skew_g(1,3) =  skew_g(2);
    T_skew_g(2,3) = -skew_g(1);
    gyro_corrected = C_mg*T_skew_g*diag(scale_g)*(dataSynced.omegaIMU + bGyr); 
    
    g_e = [0;0;-9.80665];
    g_a = C_ae*g_e;
    
    odefun = @(t,x) imuDeadReckoningODE(t, x, dataSynced.t, accel_corrected, gyro_corrected, g_a);
    x_dr = [];
    
    error_position = nan(3,length(dataSynced.t));
    error_attitude = nan(3,length(dataSynced.t));
    for lv1 = min_index:interval_size:(max_index + 1 - interval_size)
        t_span = dataSynced.t(lv1:lv1 + batch_size - 1);
        x0 = [dataSynced.r_zw_a(:,lv1);
              dataSynced.v_zwa_a(:,lv1);
              dataSynced.q_ba(:,lv1)];
        options.indices_to_normalize = 7:10;
        [~, x_dr_batch] = ode1(odefun, t_span, x0, options);
        r_zw_a_dr_batch = x_dr_batch(:,1:3).';
        q_ba_dr_batch = x_dr_batch(:,7:10).';
        q_ab_dr_batch = q_ba_dr_batch;
        q_ab_dr_batch(2:4) = -q_ab_dr_batch(2:4);
        C_ab_dr_batch = quat2dcmimag(q_ab_dr_batch.');
        C_bb = matmul3d(dataSynced.C_ba(:,:,lv1:lv1 + batch_size - 1), C_ab_dr_batch);
        error_position(:,lv1:lv1 + batch_size - 1) = ...
            dataSynced.r_zw_a(:, lv1:lv1 + batch_size - 1) - r_zw_a_dr_batch;
%         error_attitude(:,lv1:lv1 + batch_size - 1) = ...
%            dataSynced.q_ba(:,lv1:lv1 + batch_size - 1) - q_ba_dr_batch;
        error_attitude(:,lv1:lv1 + batch_size - 1) = DCM_TO_ROTVEC(C_bb);
        x_dr = [x_dr;x_dr_batch];
    end
  
    error_position(:,isStatic) = 40*error_position(:,isStatic);
    error_attitude(:,isStatic) = 40*error_attitude(:,isStatic);
    error_position = error_position(:,~isGap);
    error_attitude = error_attitude(:,~isGap);
    error_position = error_position(:,all(~isnan(error_position),1));
    error_attitude = error_attitude(:,all(~isnan(error_attitude),1));
    
    error = [error_position(:);error_attitude(:)];
end

function AB = matmul3d(A,B)
AB = zeros(size(A,1),size(B,2),size(A,3));
for lv1 = 1:size(A,3)
    AB(:,:,lv1) = A(:,:,lv1)*B(:,:,lv1);
end
end
    





