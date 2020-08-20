function [results, dataCalibrated] = calibrateImu(dataSynced, options, import_results)
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
    import_results = struct();
end
if nargin < 3
    if isempty(options)
        options = struct();
    end
    import_results = struct();
end
[tol, do_frames, do_bias, do_scale, do_skew, do_grav, params] = ...
                                                        processOptions(options);

params.end_index = params.start_index + params.max_total_states - 1;
if params.end_index > length(dataSynced.t)
    params.end_index = length(dataSynced.t);
end

[C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae] = ...
                                           processImportResults(import_results);
%% LEAST SQUARES OPTIMIZATION
% compute error vector once
[~, e_pos, e_vel, e_att] = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                                                 scale_a, scale_g, skew_a, skew_g,...
                                                 C_ae, dataSynced, params);
% Initialize figure handles.
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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GYRO CALIBRATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta = Inf;
iter = 0;
delta_cost = Inf;
cost_history = [];
while norm(delta) > tol && iter < 10 && delta_cost >  tol
    indx_counter = 1;
    
    % compute error vector
    [e, e_att] = imuGyroDeadReckoningError(C_mg, bias_g, scale_g, skew_g,...
                                                   dataSynced, params);  
    
    cost = 0.5*(e.'*e)
    
    if ~isempty(cost_history)
        delta_cost = abs((cost - cost_history(end))/(cost - cost_history(1)));
    end
    cost_history = [cost_history; cost];

    h7.YData = e_att(1,:);
    h8.YData = e_att(2,:);
    h9.YData = e_att(3,:);
    pause(0.0001)
    
    % compute Jacobians
    A = [];
    
    if do_frames
        f_Cmg = @(C) imuGyroDeadReckoningError(C, bias_g, scale_g, skew_g,...
                                                   dataSynced, params); 
        A_phi_g = complexStepJacobianLie(f_Cmg,C_mg,3,@CrossOperator,'direction','left');

        A = [A, A_phi_g];
        phi_indices = indx_counter:indx_counter+2;
        indx_counter = indx_counter + 3;
    end
    
    if do_bias
        f_bias = @(b) imuGyroDeadReckoningError(C_mg, b, scale_g, skew_g,...
                                                   dataSynced, params); 
        A_bias = complexStepJacobian(f_bias, bias_g);
        A = [A, A_bias];
        bias_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_scale
        f_scale = @(s) imuGyroDeadReckoningError(C_mg, bias_g, s, skew_g,...
                                                   dataSynced, params); 
        A_scale = complexStepJacobian(f_scale, scale_g);
        A = [A, A_scale];
        scale_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_skew
        f_skew = @(s) imuGyroDeadReckoningError(C_mg, bias_g, scale_g, s,...
                                                   dataSynced, params); 
        A_skew = complexStepJacobian(f_skew, skew_g);
        A = [A, A_skew];
        skew_indices = indx_counter:indx_counter + 2;
    end
    
    % Compute step direction
    if isempty(A)
        warning('No calibration parameters have been selected.')
        delta = 1e-16;
    else
        if iter > 10
            delta = -(A.'*A + 0.5*diag(diag(A.'*A))) \ (A.' * e);
        else
            delta = -(A.'*A) \( A.' * e);
        end
    end 
    
    % decompose and update
    if do_frames
        del_phi_g = delta(phi_indices);
        C_mg = ROTVEC_TO_DCM(-del_phi_g)*C_mg;
    end
    
    if do_bias
        del_bias_g = delta(bias_indices);
        bias_g = bias_g + del_bias_g;
    end
    
    if do_scale
        del_scale_g = delta(scale_indices);
        scale_g = scale_g + del_scale_g;
    end
    
    if do_skew
        del_skew_g = delta(skew_indices);
        skew_g = skew_g + del_skew_g;
    end
    iter = iter + 1;
end
clear del_phi_g del_bias_g del_scale_g del_skew_g
disp('Gyroscope calibration complete.')
% compute error vector
[~, e_att] = imuGyroDeadReckoningError(C_mg, bias_g, scale_g, skew_g,...
                                                   dataSynced, params);  
RMSE = sqrt((e_att(:).'*e_att(:))./length(dataSynced.t));
disp(['Attitude Estimate RMSE After Calibration (rad): ' , num2str(RMSE)])
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ACCEL CALIBRATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta = Inf;
iter = 0;
delta_cost = Inf;
cost_history = [];
while norm(delta) > tol && iter < 10 && delta_cost >  tol
    indx_counter = 1;
    
    % compute error vector
    [e, e_pos, e_vel, e_acc] = imuAccelDeadReckoningError(C_ma, bias_a,...
                                                    scale_a, skew_a,...
                                                    C_ae, dataSynced, params);
    
    cost = 0.5*(e.'*e)
    
    if ~isempty(cost_history)
        delta_cost = abs((cost - cost_history(end))/(cost - cost_history(1)));
    end
    cost_history = [cost_history; cost];

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
    
    if do_frames
        f_Cma = @(C) imuAccelDeadReckoningError(C, bias_a, scale_a, skew_a,...
                                                C_ae, dataSynced, params);
                                       
        A_phi_a = complexStepJacobianLie(f_Cma,C_ma,3,@CrossOperator,'direction','left');

        A = [A, A_phi_a];
        phi_indices = indx_counter:indx_counter+2;
        indx_counter = indx_counter + 3;
    end
    
    if do_bias
        f_bias = @(b) imuAccelDeadReckoningError(C_ma, b, scale_a, skew_a,...
                                                C_ae, dataSynced, params);
        A_bias = complexStepJacobian(f_bias, bias_a);
        A = [A, A_bias];
        bias_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_scale
        f_scale = @(s) imuAccelDeadReckoningError(C_ma, bias_a, s, skew_a,...
                                                C_ae, dataSynced, params);
                                       
        A_scale = complexStepJacobian(f_scale, scale_a);
        A = [A, A_scale];
        scale_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_skew
        f_skew = @(s) imuAccelDeadReckoningError(C_ma, bias_a, scale_a, s,...
                                                C_ae, dataSynced, params);
        A_skew = complexStepJacobian(f_skew, skew_a);
        A = [A, A_skew];
        skew_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_grav
        f_C_ae = @(C) imuAccelDeadReckoningError(C_ma, bias_a, scale_a, skew_a,...
                                                C, dataSynced, params);
        f_phi_ae = @(phi) f_C_ae(expmTaylor(CrossOperator([phi(1);phi(2);0])*C_ae));
        A_phi_ae = complexStepJacobian(f_phi_ae,[0;0]);
        A = [A, A_phi_ae];
        grav_indices = indx_counter:indx_counter + 1;
    end

    % Compute step direction
    if isempty(A)
        warning('No calibration parameters have been selected.')
        delta = 1e-16
    else
        if iter > 10
            delta = -(A.'*A + 0.5*diag(diag(A.'*A))) \ (A.' * e);
        else
            delta = -(A.'*A) \( A.' * e);
        end
    end 
    
    % decompose and update
    if do_frames
        del_phi_a = delta(phi_indices(1:3));
        C_ma = ROTVEC_TO_DCM(-del_phi_a)*C_ma;
    end
    
    if do_bias
        del_bias_a = delta(bias_indices(1:3));
        bias_a = bias_a + del_bias_a;
    end
    
    if do_scale
        del_scale_a = delta(scale_indices(1:3));
        scale_a = scale_a + del_scale_a;
    end
    
    if do_skew
        del_skew_a = delta(skew_indices(1:3));
        skew_a = skew_a + del_skew_a;
    end
    
    if do_grav
        del_phi_ae = delta(grav_indices);
        C_ae = ROTVEC_TO_DCM(-[del_phi_ae;0])*C_ae;
    end

    iter = iter + 1;
end

disp('Accelerometer/Gravity Calibration Complete')
% compute error vector
[~, e_pos, e_vel] = imuAccelDeadReckoningError(C_ma, bias_a,...
                                                    scale_a, skew_a,...
                                                    C_ae, dataSynced, params);
RMSE = sqrt((e_pos(:).'*e_pos(:))./length(dataSynced.t));
disp(['Position Estimate RMSE After Calibration (m): ' , num2str(RMSE)])
RMSE = sqrt((e_vel(:).'*e_vel(:))./length(dataSynced.t));
disp(['Velocity Estimate RMSE After Calibration (m/s): ' , num2str(RMSE)])

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
plot(cost_history)
grid on
xlabel('Iteration Number', 'Interpreter', 'Latex')
ylabel('$J$ [$\left(m/s^2\right)^2$]', 'Interpreter', 'Latex')
    
end

function [tol, do_frames, do_bias, do_scale, do_skew, do_grav, params] = ...
                                                        processOptions(options)
if isfield(options,'tolerance')
    tol = options.tolerance;
else
    tol = 1e-6;
end

if isfield(options,'frames')
    do_frames = options.frames;
else
    do_frames = true;
end

if isfield(options,'bias')
    do_bias = options.bias;
else
    do_bias = true;
end

if isfield(options,'scale')
    do_scale = options.scale;
else
    do_scale = true;
end

if isfield(options,'skew')
    do_skew = options.skew;
else
    do_skew = true;
end

if isfield(options,'grav')
    do_grav = options.grav;
else
    do_grav = true;
end

if isfield(options,'interval_size')
    params.interval_size = options.interval_size;
else
    params.interval_size = 2000;
end

if isfield(options,'batch_size')
    params.batch_size = options.batch_size;
else
    params.batch_size = 500;
end

if isfield(options,'start_index')
    params.start_index = options.start_index;
else
    params.start_index = 1;
end

if isfield(options,'max_total_states')
    params.max_total_states = options.max_total_states;
else
    params.max_total_states = 30000;
end

end

function [C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae] = ...
                                           processImportResults(import_results)
if isfield(import_results,'C_ms_accel')
    C_ma = import_results.C_ms_accel;
else
    C_ma = eye(3);
end
if isfield(import_results,'C_ms_gyro')
    C_mg = import_results.C_mg_accel;
else
    C_mg = eye(3);
end
if isfield(import_results,'bias_accel')
    bias_a = import_results.bias_accel;
else
    bias_a = [0;0;0];
end
if isfield(import_results,'bias_gyro')
    bias_g = import_results.bias_gyro;
else
    bias_g = [0;0;0];    
end
if isfield(import_results,'scale_accel')
    scale_a = import_results.scale_accel;
else
    scale_a = [1;1;1];
end
if isfield(import_results,'scale_gyro')
    scale_g = import_results.scale_gyro;
else
    scale_g = [1;1;1];
end
if isfield(import_results,'skew_accel')
    skew_a = import_results.skew_accel;
else
    skew_a = [0;0;0];
end
if isfield(import_results,'skew_gyro')
    skew_g = import_results.skew_gyro;
else
    skew_g = [0;0;0];
end
if isfield(import_results,'g_a')
    a = cross(import_results.g_a, [0;0;-9.80665]);
    a = a./norm(a);
    theta = arccos(dot(import_results.g_a, [0;0;-9.80665])/(9.80665^2));
    phi = a*theta;
    C_ae = ROTVEC_TO_DCM(phi);
else
    C_ae = eye(3);
end

% Provide an initial guess for the biases.
% if any(dataSynced.staticIndices)
%     isStatic = dataSynced.staticIndices;
%     bias_a = mean(dataSynced.accel_mocap(:,isStatic) - dataSynced.accel(:,isStatic),2);
%     bias_g = mean(dataSynced.gyro_mocap(:,isStatic) - dataSynced.gyro(:,isStatic),2);
% else
% end
end



