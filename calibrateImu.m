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
    scale = true;
end

if isfield(options,'skew')
    skew = options.skew;
else
    skew = true;
end

if isfield(options,'grav')
    grav = options.grav;
else
    grav = true;
end

%% LS Optimization on SO(3), using Gauss-Newton

% Provide an initial guess for the biases.
if any(dataSynced.staticIndices)
    isStatic = dataSynced.staticIndices;
    bias_a = mean(dataSynced.accel_mocap(:,isStatic) - dataSynced.accel(:,isStatic),2);
    bias_g = mean(dataSynced.gyro_mocap(:,isStatic) - dataSynced.gyro(:,isStatic),2);
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

params.window_size = 30000;
params.min_index = 1; 
params.interval_size = 30000; %round(length(dataSynced.t)/20);
params.batch_size = 30000;
params.max_index = params.min_index + params.window_size - 1;
if params.max_index > length(dataSynced.t)
    params.max_index = length(dataSynced.t);
end


% compute error vector once
[~, e_pos, e_vel, e_att] = errorDeadReckoning(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);

figure(1)
cla
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

figure(2)
cla
subplot(3,1,1)
h4 = plot(real(e_vel(1,:)));
axis([-inf inf -1 1])
title('Velocity Dead-Reckoning Error')
grid on
subplot(3,1,2)
h5 = plot(real(e_vel(2,:)));
axis([-inf inf -1 1])
grid on
subplot(3,1,3)
h6 = plot(real(e_vel(3,:)));
axis([-inf inf -1 1])
grid on

figure(3)
cla
subplot(3,1,1)
h7 = plot(real(e_att(1,:)));
axis([-inf inf -pi pi])
grid on
title('Attitude Dead-Reckoning Error')
subplot(3,1,2)
h8 = plot(real(e_att(2,:)));
axis([-inf inf -pi pi])
grid on
subplot(3,1,3)
h9 = plot(real(e_att(3,:)));
axis([-inf inf -pi pi])
grid on

pause(eps)

delta = Inf;
iter = 0;
delta_cost = Inf;
while norm(delta) > TOL && iter < 100 && delta_cost > TOL 
    indx_counter = 1;
    
    % compute error vector
    [e, e_pos, e_vel, e_att] = errorDeadReckoning(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
    assert(all(size(e_pos) == size(e_vel)));
    assert(all(size(e_pos) == size(e_att)));    
    
    cost = 0.5*(e.'*e)
    
    if ~isempty(costFuncHist)
        delta_cost = abs((cost - costFuncHist(end))/(cost - costFuncHist(1)));
    end
    costFuncHist = [costFuncHist; cost];
    
    
    h1.YData = e_pos(1,:);
    h2.YData = e_pos(2,:);
    h3.YData = e_pos(3,:);
    h4.YData = e_vel(1,:);
    h5.YData = e_vel(2,:);
    h6.YData = e_vel(3,:);
    h7.YData = e_att(1,:);
    h8.YData = e_att(2,:);
    h9.YData = e_att(3,:);
    pause(0.0001)
    
    % compute Jacobians
    A = [];
    
    if frames
        f_Cma = @(C) errorDeadReckoning(C, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
        A_phia = complexStepJacobianLie(f_Cma,C_ma,3,@CrossOperator,'direction','left');

        f_Cmg = @(C) errorDeadReckoning(C_ma, C, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
        A_phig = complexStepJacobianLie(f_Cmg,C_mg,3,@CrossOperator,'direction','left');

        A = [A, A_phia, A_phig];
        phi_indices = indx_counter:indx_counter+5;
        indx_counter = indx_counter + 6;
    end
    
    if bias
        f_bias = @(b) errorDeadReckoning(C_ma, C_mg, b(1:3), b(4:6), scale_a, scale_g, skew_a, skew_g, C_ae, dataSynced, params);
        A_bias = complexStepJacobian(f_bias, [bias_a;bias_g]);
        A = [A, A_bias];
        bias_indices = indx_counter:indx_counter + 5;
        indx_counter = indx_counter + 6;
    end
    
    if scale
        f_scale = @(s) errorDeadReckoning(C_ma, C_mg, bias_a, bias_g, s(1:3), s(4:6), skew_a, skew_g, C_ae, dataSynced, params);
        A_scale = complexStepJacobian(f_scale, [scale_a;scale_g]);
        A = [A, A_scale];
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
        if iter > 10
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
g_e = [0;0;-9.80665];
results.C_ms_accel = C_ma;
results.C_ms_gyro = C_mg;
results.bias_accel = bias_a;
results.bias_gyro = bias_g;
results.scale_accel = scale_a;
results.scale_gyro = scale_g;
results.skew_accel = skew_a;
results.skew_gyro = skew_g;
results.g_a = C_ae*g_e;

dataCalibrated = imuCorrectMeasurements(dataSynced, results);

% Calibrated ground truth accel/gyro measurements.
g_a = results.g_a;
mocap_gyro = dataSynced.gyro_mocap;
mocap_accel = zeros(3, length(dataSynced.t));
for lv1 = 1:length(dataSynced.t)
    mocap_accel(:,lv1) = dataSynced.accel_mocap(:,lv1) - dataSynced.C_ba(:,:,lv1)*(g_a - g_e);
end

dataCalibrated.t = dataSynced.t;
dataCalibrated.accel_mocap = mocap_accel;
dataCalibrated.gyro_mocap = mocap_gyro;

%% Plotting to evaluate performance visually
% Plot calibrated data - accelerometers
figure
sgtitle('Accelerometer')
subplot(3,1,1)
plot(dataCalibrated.t, dataCalibrated.accel(1,:))
hold on
plot(dataCalibrated.t, dataCalibrated.accel_mocap(1,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\ddot{x}$ [$m/s^2$]', 'Interpreter', 'Latex')
legend('Calibrated IMU Data', 'Mocap Data')
subplot(3,1,2)
plot(dataCalibrated.t, dataCalibrated.accel(2,:))
hold on
plot(dataCalibrated.t, dataCalibrated.accel_mocap(2,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\ddot{y}$ [$m/s^2$]', 'Interpreter', 'Latex')
subplot(3,1,3)
plot(dataCalibrated.t, dataCalibrated.accel(3,:))
hold on
plot(dataCalibrated.t, dataCalibrated.accel_mocap(3,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\ddot{z}$ [$m/s^2$]', 'Interpreter', 'Latex')

% Plot calibrated data - gyroscopes
figure
sgtitle('Gyroscope')
subplot(3,1,1)
plot(dataCalibrated.t, dataCalibrated.gyro(1,:))
hold on
plot(dataCalibrated.t, dataCalibrated.gyro_mocap(1,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\omega_x$ [rad/$s$]', 'Interpreter', 'Latex')
legend('Calibrated IMU Data', 'Mocap Data')

subplot(3,1,2)
plot(dataCalibrated.t, dataCalibrated.gyro(2,:))
hold on
plot(dataCalibrated.t, dataCalibrated.gyro_mocap(2,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\omega_y$ [rad/$s$]', 'Interpreter', 'Latex')
subplot(3,1,3)
plot(dataCalibrated.t, dataCalibrated.gyro(3,:))
hold on
plot(dataCalibrated.t, dataCalibrated.gyro_mocap(3,:))
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
function [error, error_position, error_velocity, error_attitude] = ...
    errorDeadReckoning(...
    C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae,...
    dataSynced, params ...
    ) 
% ERRORDEADRECKONING Corrects the IMU with the provided calibration
% parameters, then integrates the data forward on specific intervals.
% Returns the error between the integrated solution and the ground truth.
    
    % Extract some relevant information and parameters.
    isGap = dataSynced.gapIndices;
    isStatic = dataSynced.staticIndices;   
    interval_size = params.interval_size;
    batch_size = params.batch_size;
    min_index = params.min_index;
    max_index = params.max_index;
    
    calib_params.C_ms_accel = C_ma;
    calib_params.C_ms_gyro = C_mg;
    calib_params.bias_accel = bias_a;
    calib_params.bias_gyro = bias_g;
    calib_params.scale_accel = scale_a;
    calib_params.scale_gyro = scale_g;
    calib_params.skew_accel = skew_a;
    calib_params.skew_gyro = skew_g;
   
    data_corrected = imuCorrectMeasurements(dataSynced, calib_params);
    
    % Corrected gravity in the mocap world frame.
    g_e = [0;0;-9.80665];
    g_a = C_ae*g_e;
    
    % Go through each interval and dead-reckon for a small duration of
    % length batch_size. Compare results to ground truth.
    error_position = nan(3,length(dataSynced.t));
    error_velocity = nan(3,length(dataSynced.t));
    error_attitude = nan(3,length(dataSynced.t));
    error_accel = nan(3,length(dataSynced.t));
    error_omega = nan(3,length(dataSynced.t));
    for lv1 = min_index:interval_size:(max_index + 1 - interval_size)        
        N = batch_size;
        idx = lv1:lv1 + N -1;
        idx = idx(:);
        isStaticInBatch = idx(:).*isStatic(idx);
        isStaticInBatch = isStaticInBatch(isStaticInBatch ~= 0);
        
        % Initial conditions from mocap (if we detected static, force zero
        % velocity).
        r_zw_a_0 = dataSynced.r_zw_a(:,lv1);
        if isStatic(lv1)
            v_zwa_a_0 = zeros(3,1);
        else
            v_zwa_a_0 = dataSynced.v_zwa_a(:,lv1);
        end
        C_ba_0 = dataSynced.C_ba(:,:,lv1);
        t_span = dataSynced.t(idx);
        traj_dr = imuDeadReckoning(data_corrected, r_zw_a_0, v_zwa_a_0, C_ba_0,...
                                   g_a, 'so3',t_span);
        
        % Build errors. If static, force zero as the reference. 
        error_position(:,idx) = dataSynced.r_zw_a(:,idx) - traj_dr.r_zw_a;
        error_velocity(:,idx) = dataSynced.v_zwa_a(:,idx) - traj_dr.v_zwa_a;
        error_velocity(:,isStaticInBatch) = error_velocity(:,isStaticInBatch) ...
                                        - dataSynced.v_zwa_a(:,isStaticInBatch);
        error_accel(:,idx) = dataSynced.a_zwa_a(:,idx) - traj_dr.a_zwa_a;
        error_omega(:,idx) = dataSynced.gyro_mocap(:,idx) - data_corrected.gyro(:,idx);
        for lv2 = 1:N
            % Index in the actual dataSynced timeseries.
            idxData = lv2 + lv1 -1;
            error_attitude(:,idxData) = DCM_TO_ROTVEC((traj_dr.C_ba(:,:,lv2)...
                                              *dataSynced.C_ba(:,:,idxData).'));
        end

    end
    
    % Add weight when static. 
    error_position(:,isStatic) = error_position(:,isStatic);
    error_velocity(:,isStatic) = 100*error_velocity(:,isStatic);
    error_attitude(:,isStatic) = error_attitude(:,isStatic);
    
    
    % Discard any corresponding to gaps in the mocap data, as the spline is
    % potentially inaccurate during these times. 
    error_position = error_position(:,~isGap);
    error_velocity = error_velocity(:,~isGap);
    error_attitude = error_attitude(:,~isGap);
    error_accel = error_accel(:,~isGap);
    error_omega = error_omega(:,~isGap);
    
    % Remove any NANs, which are periods for which we are not
    % dead-reckoning.
    % Warning: potential silent failing here if the indexing is not done
    % properly.... NANs will just get deleted.
    error_position = error_position(:,all(~isnan(error_position),1));
    error_velocity = error_velocity(:,all(~isnan(error_velocity),1));
    error_attitude = error_attitude(:,all(~isnan(error_attitude),1));
    error_accel = error_accel(:,all(~isnan(error_attitude),1));
    error_omega = error_omega(:,all(~isnan(error_attitude),1));
    
    phi_ma = DCM_TO_ROTVEC(C_ma);
    phi_mg = DCM_TO_ROTVEC(C_mg);
    error_regularize = [phi_ma;
                        phi_mg;
                        0.1*bias_a;
                        bias_g;
                        ones(3,1) - scale_a;
                        ones(3,1) - scale_g;
                        skew_a;
                        skew_g];
                        
                        
    error = [
             error_position(:);
             error_velocity(:);
             10*error_attitude(:);
             error_accel(:);
             error_omega(:);
             %sqrt(size(error_position,2))*error_regularize
             ];
end

% function AB = matmul3d(A,B)
% AB = zeros(size(A,1),size(B,2),size(A,3));
% for lv1 = 1:size(A,3)
%     AB(:,:,lv1) = A(:,:,lv1)*B(:,:,lv1);
% end
% end
    





