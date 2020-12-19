function [results, data_calibrated] = posCalibrate(data_synced, data_pos,...
                                                    options, import_results)
% TODO: 1) explicitly mention how time is synced! (we need the original timestamps)
%           ---> assumption of same clock as IMU
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
%       r_zw_m: [3 x N] double
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
%       bias_sccel: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       bias_gyro: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       scale_sccel: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       scale_gyro: (optional) boolean
%           Toggle true/false to override accel sensorframe calibration.
%       skew_sccel: (optional) boolean
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
%       bias_sccel: (optional) [3 x 1] double
%           Accelerometer bias
%       bias_gyro: (optional) [3 x 1] double
%           Gyroscope bias
%       scale_sccel: (optional) [3 x 1] double
%           Accelerometer scale factor
%       scale_gyro: (optional) [3 x 1] double
%           Gyroscope scale factor
%       skew_sccel: (optional) [3 x 1] double
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

if nargin < 2
    error('DECAR_MOCAP_TOOLS: Missing data in posCalibrate.')
end
if nargin < 3
    options = struct();
    import_results = struct();
end
if nargin < 4
    if isempty(options)
        options = struct();
    end
    import_results = struct();
end
[tol, do_receiver_pos, do_frame, do_bias, do_scale, do_skew, params] ...
                                            = processOptions(options)

% params.end_index = params.start_index + params.max_total_states - 1;
% if params.end_index > length(data_synced.t)
%     params.end_index = length(data_synced.t);
% end

[r_pz_b, C_ms, bias_s, scale_s, skew_s] = processImportResults(import_results);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% POS CALIBRATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Find the name of the field with the position data
all_fields = fieldnames(data_pos);
field_name = all_fields(contains(all_fields, 'pos'));

if numel(field_name) > 1
    error(['DECAR_MOCAP_TOOLS: More than one set of position measurements',...
            ' detected. Please calibrate them one at a time.'])
else
    field_name = field_name{1};
end

% Synced time stamps
t_s = data_pos.time - data_synced.t_0 + data_synced.t(1);
t_end = data_synced.t(end);
idx_keep = (t_s>0 & t_s<t_end);
t_s = t_s(idx_keep);

% Uncalibrated, synced position measurements
r_pw_s = data_pos.(field_name);
r_pw_s = r_pw_s(:,idx_keep);

% Initialize the output
num_meas = numel(data_pos.time);
for lv1=1:numel(all_fields)
    temp = data_pos.(all_fields{lv1});
    idx_t = find(size(temp) == num_meas);
    switch num2str(idx_t)
        case '1'
            data_calibrated.(all_fields{lv1}) = temp(idx_keep);
        case '2'
            data_calibrated.(all_fields{lv1}) = temp(:,idx_keep);
        case '3'
            data_calibrated.(all_fields{lv1}) = temp(:,:,idx_keep);
        otherwise
            data_calibrated.(all_fields{lv1}) = temp;
    end
end
data_calibrated.t = t_s;
    
% Ground truth
spline_mocap = mocapGetSplineProperties(data_synced, 1);
waypoints = ppval(spline_mocap, t_s);
r_zw_m = waypoints(1:3,:);
C_bm = quat2dcm(waypoints(4:7,:)');
C_mb = permute(C_bm, [2, 1, 3]);

delta = Inf;
iter = 0;
delta_cost = Inf;
cost_history = [];
while norm(delta) > tol && iter < 10 && delta_cost >  tol
    indx_counter = 1;
    
    % compute error vector
    e_pos = posMeasError(r_pz_b, C_ms, bias_s, scale_s, skew_s,...
                         r_pw_s, r_zw_m, C_mb, params);
    
    cost = 0.5*(e_pos.'*e_pos)
    
    if ~isempty(cost_history)
        delta_cost = abs((cost - cost_history(end))/(cost - cost_history(1)));
    end
    cost_history = [cost_history; cost];
    
    % compute Jacobians
    A = [];
    
    if do_receiver_pos
        f_r_pz = @(r) posMeasError(r, C_ms, bias_s, scale_s, skew_s,...
                                   r_pw_s, r_zw_m, C_mb, params);
        A_r_pz = complexStepJacobian(f_r_pz, r_pz_b);
        A = [A, A_r_pz];
        r_pz_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_frame
        f_Cma = @(C) posMeasError(r_pz_b, C, bias_s, scale_s, skew_s,...
                                  r_pw_s, r_zw_m, C_mb, params);
                                       
        A_phi_a = complexStepJacobianLie(f_Cma, C_ms, 3, @CrossOperator,...
                                         'direction', 'left');

        A = [A, A_phi_a];
        phi_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_bias
        f_bias = @(b) posMeasError(r_pz_b, C_ms, b, scale_s, skew_s,...
                                   r_pw_s, r_zw_m, C_mb, params);
        A_bias = complexStepJacobian(f_bias, bias_s);
        A = [A, A_bias];
        bias_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_scale
        f_scale = @(s) posMeasError(r_pz_b, C_ms, bias_s, s, skew_s,...
                                    r_pw_s, r_zw_m, C_mb, params);
                                       
        A_scale = complexStepJacobian(f_scale, scale_s);
        A = [A, A_scale];
        scale_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_skew
        f_skew = @(s) posMeasError(r_pz_b, C_ms, bias_s, scale_s, s,...
                                   r_pw_s, r_zw_m, C_mb, params);
        A_skew = complexStepJacobian(f_skew, skew_s);
        A = [A, A_skew];
        skew_indices = indx_counter:indx_counter + 2;
    end

    % Compute step direction
    if isempty(A)
        warning('No calibration parameters have been selected.')
        delta = 1e-16
    else
        if iter > 10
            delta = -(A.'*A + 0.5*diag(diag(A.'*A))) \ (A.' * e_pos);
        else
            delta = -(A.'*A) \( A.' * e_pos);
        end
    end 
    
    % decompose and update
    if do_receiver_pos
        del_r_pz = delta(r_pz_indices(1:3));
        r_pz_b = r_pz_b + del_r_pz;
    end
    
    if do_frame
        del_phi_a = delta(phi_indices(1:3));
        C_ms = ROTVEC_TO_DCM(-del_phi_a)*C_ms;
    end
    
    if do_bias
        del_bias_s = delta(bias_indices(1:3));
        bias_s = bias_s + del_bias_s;
    end
    
    if do_scale
        del_scale_s = delta(scale_indices(1:3));
        scale_s = scale_s + del_scale_s;
    end
    
    if do_skew
        del_skew_s = delta(skew_indices(1:3));
        skew_s = skew_s + del_skew_s;
    end

    iter = iter + 1;
end

disp('Accelerometer/Gravity Calibration Complete')
% compute error vector
e_pos = posMeasError(r_pz_b, C_ms, bias_s, scale_s, skew_s,...
                     r_pw_s, r_zw_m, C_mb, params);
RMSE = sqrt((e_pos(:).'*e_pos(:))./length(t_s));
disp(['Position Measurement RMSE After Calibration (m): ' , num2str(RMSE)])

%% Results and Output
results.r_pz_b = r_pz_b;
results.C_ms = C_ms;
results.bias = bias_s;
results.scale = scale_s;
results.skew = skew_s;

% Store the calibrated position measurements
T_skew = eye(3);
T_skew(1,2) = -skew_s(3);
T_skew(1,3) =  skew_s(2);
T_skew(2,3) = -skew_s(1);
data_calibrated.(field_name) = C_ms*T_skew*diag(scale_s)*(r_pw_s + bias_s);

%% Plotting to evaluate performance visually
% Plot calibrated data - accelerometers
figure
sgtitle('Position Measurements')
subplot(3,1,1)
plot(data_calibrated.t, data_calibrated.(field_name)(1,:))
hold on
plot(data_calibrated.t, r_zw_m(1,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$x$ [$m$]', 'Interpreter', 'Latex')
legend('Calibrated Position Measurements', 'Mocap Data')
subplot(3,1,2)
plot(data_calibrated.t, data_calibrated.(field_name)(2,:))
hold on
plot(data_calibrated.t, r_zw_m(2,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$y$ [$m$]', 'Interpreter', 'Latex')
subplot(3,1,3)
plot(data_calibrated.t, data_calibrated.(field_name)(3,:))
hold on
plot(data_calibrated.t, r_zw_m(3,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$z$ [$m$]', 'Interpreter', 'Latex')

% Plot evolution of cost function
figure
plot(cost_history)
grid on
xlabel('Iteration Number', 'Interpreter', 'Latex')
ylabel('$J$ [$m^2$]', 'Interpreter', 'Latex')  
end

%% ERROR COMPUTING
function e = posMeasError(r_pz_b, C_ms, bias_s, scale_s, skew_s,...
                          r_pw_s, r_zw_m, C_mb, params)
    e = zeros(3,size(r_zw_m,2));
    
    T_skew = eye(3);
    T_skew(1,2) = -skew_s(3);
    T_skew(1,3) =  skew_s(2);
    T_skew(2,3) = -skew_s(1);
    
    r_pw_m_hat = C_ms*T_skew*diag(scale_s)*(r_pw_s + bias_s);
    for lv1=1:size(r_zw_m,2)
        r_pw_m_iter = r_zw_m(:,lv1) + C_mb(:,:,lv1) * r_pz_b;
        e(:,lv1) = r_pw_m_iter - r_pw_m_hat(:,lv1);
    end
    e = e(:);
end
   
%% OPTIONS PROCESSING
function [tol, do_receiver_pos, do_frame, do_bias, do_scale, do_skew, params]...
                                                       = processOptions(options)
if isfield(options,'tolerance')
    tol = options.tolerance;
else
    tol = 1e-6;
end

if isfield(options,'receiver_pos')
    do_receiver_pos = options.receiver_pos;
else
    do_receiver_pos = false;
end

if isfield(options,'frame')
    do_frame = options.frame;
else
    do_frame = true;
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

% TODO
params = [];
% if isfield(options,'interval_size')
%     params.interval_size = options.interval_size;
% else
%     params.interval_size = 2000;
% end
% 
% if isfield(options,'batch_size')
%     params.batch_size = options.batch_size;
% else
%     params.batch_size = 500;
% end
% 
% if isfield(options,'start_index')
%     params.start_index = options.start_index;
% else
%     params.start_index = 1;
% end
% 
% if isfield(options,'max_total_states')
%     params.max_total_states = options.max_total_states;
% else
%     params.max_total_states = 30000;
% end

end
%% IMPORT RESULTS PROCESSING
function [r_pz_b, C_ms, bias_s, scale_s, skew_s] = processImportResults(import_results)

if isfield(import_results,'r_pz_b')
    r_pz_b = import_results.r_pz_b;
else
    r_pz_b = [0;0;0];
end
if isfield(import_results,'C_ms')
    C_ms = import_results.C_ms;
else
    C_ms = eye(3);
end
if isfield(import_results,'bias')
    bias_s = import_results.bias;
else
    bias_s = [0;0;0];
end
if isfield(import_results,'scale')
    scale_s = import_results.scale;
else
    scale_s = [1;1;1];
end
if isfield(import_results,'skew')
    skew_s = import_results.skew;
else
    skew_s = [0;0;0];
end

end



