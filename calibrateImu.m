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
    TOL = 1e-9;
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

if isfield(options,'interval_size')
    params.interval_size = options.interval_size;
else
    params.interval_size = 30000;
end

if isfield(options,'batch_size')
    params.batch_size = options.batch_size;
else
    params.batch_size = 30000;
end

if isfield(options,'start_index')
    params.start_index = options.start_index;
else
    params.start_index = 1; %round(length(dataSynced.t)/20);
end

if isfield(options,'max_total_states')
    params.max_total_states = options.max_total_states;
else
    params.max_total_states = 30000; %round(length(dataSynced.t)/20);
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

params.end_index = params.start_index + params.max_total_states - 1;
if params.end_index > length(dataSynced.t)
    params.end_index = length(dataSynced.t);
end


% compute error vector once
[~, e_pos, e_vel, e_att] = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                                                 scale_a, scale_g, skew_a, skew_g,...
                                                 C_ae, dataSynced, params);

figure(1)
cla
subplot(3,1,1)
h1 = plot(real(e_pos(1,:)));
%axis([-inf inf -1 1])
title('Position Dead-Reckoning Error')
grid on
subplot(3,1,2)
h2 = plot(real(e_pos(2,:)));
%axis([-inf inf -1 1])
grid on
subplot(3,1,3)
h3 = plot(real(e_pos(3,:)));
%axis([-inf inf -1 1])
grid on

figure(2)
cla
subplot(3,1,1)
h4 = plot(real(e_vel(1,:)));
%axis([-inf inf -1 1])
title('Velocity Dead-Reckoning Error')
grid on
subplot(3,1,2)
h5 = plot(real(e_vel(2,:)));
%axis([-inf inf -1 1])
grid on
subplot(3,1,3)
h6 = plot(real(e_vel(3,:)));
%axis([-inf inf -1 1])
grid on

figure(3)
cla
subplot(3,1,1)
h7 = plot(real(e_att(1,:)));
%axis([-inf inf -pi pi])
grid on
title('Attitude Dead-Reckoning Error')
subplot(3,1,2)
h8 = plot(real(e_att(2,:)));
%axis([-inf inf -pi pi])
grid on
subplot(3,1,3)
h9 = plot(real(e_att(3,:)));
%axis([-inf inf -pi pi])
grid on

pause(eps)

delta = Inf;
iter = 0;
delta_cost = Inf;
while norm(delta) > TOL && iter < 100 && delta_cost >  TOL
    indx_counter = 1;
    
    % compute error vector
    [e, e_pos, e_vel, e_att] = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                                                    scale_a, scale_g, skew_a, skew_g,...
                                                    C_ae, dataSynced, params);
    %assert(all(size(e_pos) == size(e_vel)));
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
        f_Cma = @(C) imuDeadReckoningError(C, C_mg, bias_a, bias_g,...
                                           scale_a, scale_g, skew_a, skew_g,...
                                           C_ae, dataSynced, params);
                                       
        A_phia = complexStepJacobianLie(f_Cma,C_ma,3,@CrossOperator,'direction','left');

        f_Cmg = @(C) imuDeadReckoningError(C_ma, C, bias_a, bias_g,...
                                           scale_a, scale_g, skew_a, skew_g,...
                                           C_ae, dataSynced, params);
        A_phig = complexStepJacobianLie(f_Cmg,C_mg,3,@CrossOperator,'direction','left');

        A = [A, A_phia, A_phig];
        phi_indices = indx_counter:indx_counter+5;
        indx_counter = indx_counter + 6;
    end
    
    if bias
        f_bias = @(b) imuDeadReckoningError(C_ma, C_mg, b(1:3), b(4:6),...
                                            scale_a, scale_g, skew_a, skew_g,...
                                            C_ae, dataSynced, params);
        A_bias = complexStepJacobian(f_bias, [bias_a;bias_g]);
        A = [A, A_bias];
        bias_indices = indx_counter:indx_counter + 5;
        indx_counter = indx_counter + 6;
    end
    
    if scale
        f_scale = @(s) imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                                             s(1:3), s(4:6), skew_a, skew_g,...
                                             C_ae, dataSynced, params);
        A_scale = complexStepJacobian(f_scale, [scale_a;scale_g]);
        A = [A, A_scale];
        scale_indices = indx_counter:indx_counter + 5;
        indx_counter = indx_counter + 6;
    end
    
    if skew
        f_skew = @(s) imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                                            scale_a, scale_g, s(1:3), s(4:6),...
                                            C_ae, dataSynced, params);
        A_skew = complexStepJacobian(f_skew, [skew_a; skew_g]);
        A = [A, A_skew];
        skew_indices = indx_counter:indx_counter + 5;
        indx_counter = indx_counter + 6;
    end
    
    if grav
        f_Cae = @(C) imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                                           scale_a, scale_g, skew_a, skew_g,...
                                           C, dataSynced, params);
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
        del_phi_a = delta(phi_indices(1:3));
        del_phi_g = delta(phi_indices(4:6));
        C_ma = ROTVEC_TO_DCM(-del_phi_a)*C_ma;
        C_mg = ROTVEC_TO_DCM(-del_phi_g)*C_mg;
    end
    
    if bias
        del_bias_a = delta(bias_indices(1:3));
        del_bias_g = delta(bias_indices(4:6));
        bias_a = bias_a + del_bias_a;
        bias_g = bias_g + del_bias_g;
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
        del_phi_ae = delta(grav_indices);
        C_ae = ROTVEC_TO_DCM(-[del_phi_ae;0])*C_ae;
    end

    iter = iter + 1;
end

% compute error vector
e = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g, scale_a, scale_g,...
                          skew_a, skew_g, C_ae, dataSynced, params);
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






