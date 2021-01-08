function [results, data_calibrated] = imuCalibrate2(data_synced, options, import_results)
%IMUCALIBRATE Determines the calibration parameters of the accelerometer
% and gyroscope, which are the sensorframe-to-bodyframe DCMs, biases, scale
% factors, axis misalignments ("skew"), and mocap local frame gravity
% misalignment. This is done by minimizing the dead-reckoning error.
%
% Example uses:
%       results = imuCalibrate(data_synced)
%       results = imuCalibrate(data_synced, options)
%       results = imuCalibrate(data_synced, options, import_results)
%       results = imuCalibrate(data_synced, [], import_results)
%       [results, data_calibrated] = imuCalibrate(data_synced)
%
% PARAMETERS:
% -----------
% data_synced: struct with fields (output of imuMocapSyncTime())
%       t: [N x 1] double
%           imu timestamps in the mocap clock reference
%       accel: [3 x N] double
%           accelerometer raw data
%       gyro: [3 x N] double
%           gyroscope raw data
%       accel_mocap: [3 x N] double
%           fake accelerometer measurements from ground truth
%       gyro_mocap: [3 x N] double
%           fake gyroscope measurements from ground truth
%       r_zw_a: [3 x N] double
%           ground truth position of IMU in mocap local frame
%       C_ba: [3 x 3 x N] double
%           ground truth attitude of mocap body frame.
% options: (optional) struct 
%   User can pass a struct with a subset of any of the following fields
%   to toggle on/off different calibration parameters. If a field is
%   not included in the options struct, a default value will take its
%       place.
%       frames: (optional) boolean
%           Default true. Toggle true/false to calibrate accel/gyro body  
%           frame to sensor frame DCMs
%       bias: (optional) boolean
%           Default true. Toggle true/false to calibrate accel/gyro biases.
%       scale: (optional) boolean
%           Default true. Toggle true/false to calibrate accel/gyro scale 
%           factors.
%       skew: (optional) boolean
%           Default true. Toggle true/false to calibrate accel/gyro axis 
%           misalignments.
%       grav: (optional) boolean
%           Default true. Toggle true/false to calibrate mocap local frame 
%           gravity vector.
%   The following options have priority over the previous. 
%       frame_accel: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       frame_gyro: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       bias_accel: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       bias_gyro: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       scale_accel: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       scale_gyro: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       skew_accel: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       skew_gyro: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
% import_results: (optional) struct
%   Values for the calibration parameters to either use as an initial
%   guess if that specific parameter is being calibrated, or to use as
%   a fixed value if it is not being calibrated.
%       C_ms_accel: (optional) [3 x 3] double
%           DCM between mocap body frame and accel sensor frame.
%       C_ms_accel: (optional) [3 x 3] double
%           DCM between mocap body frame and gyro sensor frame.
%       bias_accel: (optional) [3 x 1] double
%           Accelerometer bias
%       bias_gyro: (optional) [3 x 1] double
%           Gyroscope bias
%       scale_accel: (optional) [3 x 1] double
%           Accelerometer scale factor
%       scale_gyro: (optional) [3 x 1] double
%           Gyroscope scale factor
%       skew_accel: (optional) [3 x 1] double
%           Accelerometer axis misalignments
%       skew_gyro: (optional) [3 x 1] double
%           Gyroscope axis misalignments
%       g_a: (optional) [3 x 1] double
%           Gravity vector in frame a, the mocap local frame.
% 
% RETURNS:
% ---------
% results: struct
%       has the same fields and descriptions as import_results
% data_calibrated: struct
%       has the same fields and description as data_synced except for the
%       following fields
%       accel: [3 x N] double
%           CALIBRATED accelerometer measurements.
%       gyro: [3 x N] double
%           CALIBRATED gyroscope measurements.

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
[tol, do_frame_accel, do_bias_accel, do_scale_accel, do_skew_accel,...
do_frame_gyro, do_bias_gyro, do_scale_gyro, do_skew_gyro, do_grav, params]...
                                                       = processOptions(options);

params.end_index = params.start_index + params.max_total_states - 1;
if params.end_index > length(data_synced.t)
    params.end_index = length(data_synced.t);
end

[C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae] = ...
                                           processImportResults(import_results);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GYRO CALIBRATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

variables(1).x_0 = C_mg;
variables(1).update_func = @(X,dx)X*expm(crossOp(dx));
variables(1).dimension = 3;
variables(1).disabled = ~do_frame_gyro;

variables(2).x_0 = bias_g;
variables(2).disabled = ~do_bias_gyro;

variables(3).x_0 = scale_g;
variables(3).disabled = ~do_scale_gyro;
    
variables(4).x_0 = skew_g;
variables(4).disabled = ~do_skew_gyro;

err_func = @(x) imuGyroDeadReckoningError2(x, data_synced, params);  

x_opt = leastSquares(err_func, variables);

C_mg = x_opt{1};
bias_g = x_opt{2};
scale_g = x_opt{3};
skew_g = x_opt{4};


[~, e_att] = imuGyroDeadReckoningError(C_mg, bias_g, scale_g, skew_g,...
                                                   data_synced, params);  % Todo. remove.
RMSE = sqrt((e_att(:).'*e_att(:))./length(data_synced.t));
disp(['Attitude Estimate RMSE After Calibration (rad): ' , num2str(RMSE)])
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ACCEL CALIBRATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables err_func
% Provide an initial guess for the biases.
if any(data_synced.staticIndices)
    isStatic = data_synced.staticIndices;
    bias_a = mean(data_synced.accel_mocap(:,isStatic) - data_synced.accel(:,isStatic),2);
end

variables(1).x_0 = C_ma;
variables(1).update_func = @(X,dx)expm(crossOp(dx))*X;
variables(1).dimension = 3;
variables(1).disabled = ~do_frame_accel;

variables(2).x_0 = bias_a;
variables(2).disabled = ~do_bias_accel;

variables(3).x_0 = scale_a;
variables(3).disabled = ~do_scale_accel;
    
variables(4).x_0 = skew_a;
variables(4).disabled = ~do_skew_accel;

variables(5).x_0 = C_ae;
variables(5).update_func = @(X,dx) X*expm(crossOp([dx(1);dx(2);0]));
variables(5).dimension = 2;
variables(5).disabled = ~do_grav;
err_func = @(x) imuAccelDeadReckoningError2(x, data_synced, params);  

x_opt = leastSquares(err_func, variables);

C_ma = x_opt{1};
bias_a = x_opt{2};
scale_a = x_opt{3};
skew_a = x_opt{4};
C_ae = x_opt{5};

% compute error vector
[~, e_pos, e_vel] = imuAccelDeadReckoningError(C_ma, bias_a,...
                                                    scale_a, skew_a,...
                                                    C_ae, data_synced, params);
RMSE = sqrt((e_pos(:).'*e_pos(:))./length(data_synced.t));
disp(['Position Estimate RMSE After Calibration (m): ' , num2str(RMSE)])
RMSE = sqrt((e_vel(:).'*e_vel(:))./length(data_synced.t));
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

data_calibrated = imuCorrectMeasurements(data_synced, results);

% Calibrated ground truth accel/gyro measurements.
g_a = results.g_a;
mocap_gyro = data_synced.gyro_mocap;
mocap_accel = zeros(3, length(data_synced.t));
for lv1 = 1:length(data_synced.t)
    mocap_accel(:,lv1) = data_synced.accel_mocap(:,lv1) - data_synced.C_ba(:,:,lv1)*(g_a - g_e);
end

data_calibrated.t = data_synced.t;
data_calibrated.accel_mocap = mocap_accel;
data_calibrated.gyro_mocap = mocap_gyro;

plotScript(data_calibrated)

end

%% OPTIONS PROCESSING
function [tol, do_frame_accel, do_bias_accel, do_scale_accel, do_skew_accel,...
          do_frame_gyro, do_bias_gyro, do_scale_gyro, do_skew_gyro, do_grav,...
          params] = processOptions(options)
if isfield(options,'tolerance')
    tol = options.tolerance;
else
    tol = 1e-6;
end

if isfield(options,'frames')
    do_frame_accel = options.frames;    
    do_frame_gyro = options.frames;
else
    do_frame_accel = true;
    do_frame_gyro = true;
end

if isfield(options,'bias')
    do_bias_accel = options.bias;
    do_bias_gyro = options.bias;
else
    do_bias_accel = true;
    do_bias_gyro = true;
end

if isfield(options,'scale')
    do_scale_accel = options.scale;
    do_scale_gyro = options.scale;
else
    do_scale_accel = true;
    do_scale_gyro = true;
end

if isfield(options,'skew')
    do_skew_accel = options.skew;
    do_skew_gyro = options.skew;
else
    do_skew_accel = true;
    do_skew_gyro = true;
end

if isfield(options,'grav')
    do_grav = options.grav;
else
    do_grav = true;
end

if isfield(options,'frame_accel')
    do_frame_accel = options.frame_accel;
end

if isfield(options,'frame_gyro')
    do_frame_gyro = options.frame_gyro;
end

if isfield(options,'bias_accel')
    do_bias_accel = options.bias_accel;
end

if isfield(options,'bias_gyro')
    do_bias_gyro = options.bias_gyro;
end

if isfield(options,'scale_accel')
    do_scale_accel = options.scale_accel;
end

if isfield(options,'scale_gyro')
    do_scale_gyro = options.scale_gyro;
end

if isfield(options,'skew_accel')
    do_skew_accel = options.skew_accel;
end

if isfield(options,'skew_gyro')
    do_skew_gyro = options.skew_gyro;
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
%% IMPORT RESULTS PROCESSING
function [C_ma, C_mg, bias_a, bias_g, scale_a, scale_g, skew_a, skew_g, C_ae] = ...
                                           processImportResults(import_results)
if isfield(import_results,'C_ms_accel')
    C_ma = import_results.C_ms_accel;
else
    C_ma = eye(3);
end
if isfield(import_results,'C_ms_gyro')
    C_mg = import_results.C_ms_accel;
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
    g_e =  [0;0;-9.80665];
    a = cross(import_results.g_a, g_e);
    a = a./norm(a);
    theta = acos(dot(import_results.g_a, g_e)/(9.80665^2));
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

function plotScript(data_calibrated)
%% Plotting to evaluate performance visually
% Plot calibrated data - accelerometers
figure
sgtitle('Accelerometer')
subplot(3,1,1)
plot(data_calibrated.t, data_calibrated.accel(1,:))
hold on
plot(data_calibrated.t, data_calibrated.accel_mocap(1,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\ddot{x}$ [$m/s^2$]', 'Interpreter', 'Latex')
legend('Calibrated IMU Data', 'Mocap Data')
subplot(3,1,2)
plot(data_calibrated.t, data_calibrated.accel(2,:))
hold on
plot(data_calibrated.t, data_calibrated.accel_mocap(2,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\ddot{y}$ [$m/s^2$]', 'Interpreter', 'Latex')
subplot(3,1,3)
plot(data_calibrated.t, data_calibrated.accel(3,:))
hold on
plot(data_calibrated.t, data_calibrated.accel_mocap(3,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\ddot{z}$ [$m/s^2$]', 'Interpreter', 'Latex')

% Plot calibrated data - gyroscopes
figure
sgtitle('Gyroscope')
subplot(3,1,1)
plot(data_calibrated.t, data_calibrated.gyro(1,:))
hold on
plot(data_calibrated.t, data_calibrated.gyro_mocap(1,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\omega_x$ [rad/$s$]', 'Interpreter', 'Latex')
legend('Calibrated IMU Data', 'Mocap Data')

subplot(3,1,2)
plot(data_calibrated.t, data_calibrated.gyro(2,:))
hold on
plot(data_calibrated.t, data_calibrated.gyro_mocap(2,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\omega_y$ [rad/$s$]', 'Interpreter', 'Latex')
subplot(3,1,3)
plot(data_calibrated.t, data_calibrated.gyro(3,:))
hold on
plot(data_calibrated.t, data_calibrated.gyro_mocap(3,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\omega_z$ [rad/$s$]', 'Interpreter', 'Latex')
end
