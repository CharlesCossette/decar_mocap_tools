function [results, data_calibrated, RMSE] = magCalibrate(data_synced, options, import_results)
%MAGCALIBRATE Calibrates a magnetometer by finding the optimal biases, 
% rotations, axis misalignments, and local magnetic field vector .
% Requires the complexStepJacobianLie code from decar_utils.

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

% SCALE UP RESULTS TO uT 
is_si_units = false;
if mean(vecnorm(data_synced.mag)) < 1e-2
    % We will assume it was provided in Teslas instead of micro Teslas
    % Convert to micro teslas for numirical stability of least squares
    data_synced.mag = data_synced.mag*(1e6);
    is_si_units = true;
end

[tol, do_frame, do_bias, do_scale, do_skew, do_mag_vector] = processOptions(options);
[m_a, C_ms_mag, bias_mag, scale_mag, skew_mag] =  processImportResults(import_results);

%% LS Optimization on SO(3), using Gauss-Newton

% Initial guess for m_a
% is_static = data_synced.staticIndices;
% m_a_data = zeros(3, numel(data_synced.t));
% for lv1=1:1:length(data_synced.t)
%     m_a_data(:,lv1) = data_synced.C_ba(:,:,lv1).'*data_synced.mag(:,lv1);
% end
% m_a = mean(m_a_data(:,~is_static),2,'omitnan');

% initialize
cost_history = [];
delta = Inf;
iter = 0;
while norm(delta) > tol && iter < 100
    % compute error vector
    e = errorMag(C_ms_mag, m_a, bias_mag, skew_mag, data_synced);
    
    % compute Jacobians
    A = [];
    indx_counter = 1;
    if do_frame
        f_C_ms_mag = @(C) errorMag(C, m_a, bias_mag, skew_mag, data_synced);
        A_phi_mag = complexStepJacobianLie(f_C_ms_mag,C_ms_mag,...
            3,@CrossOperator,'direction','right');
        A = [A, A_phi_mag];
        phi_indices = indx_counter:indx_counter+2;
        indx_counter = indx_counter + 3;
    end
    
    if do_mag_vector
        f_m_a = @(m) errorMag(C_ms_mag, m, bias_mag, skew_mag, data_synced);
        A_m_a = complexStepJacobian(f_m_a, m_a);
        A = [A, A_m_a];
        vector_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_bias
        f_bias = @(b) errorMag(C_ms_mag, m_a, b, skew_mag, data_synced);
        A_bias = complexStepJacobian(f_bias, bias_mag);
        A = [A, A_bias];
        bias_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_skew
        f_skew = @(skew) errorMag(C_ms_mag, m_a, bias_mag, skew, data_synced);
        A_skew = complexStepJacobian(f_skew, skew_mag);
        A = [A, A_skew];
        skew_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    cost = 0.5*(e.'*e)
    cost_history = [cost_history; cost];
    
    % Compute step direction
    if isempty(A)
        warning('No calibration parameters have been selected.')
        delta = 1e-16;
    else
        if iter > 100
            delta = -(A.'*A + 0.2*diag(diag(A.'*A))) \ (A.' * e);
        else
            delta = -(A.'*A) \( A.' * e);
        end
    end 
    
    % decompose and update
    if do_frame
        del_phi_mag = delta(phi_indices);
        C_ms_mag = C_ms_mag*expm(crossOp(del_phi_mag));
    end
    if do_mag_vector
        del_m_a = delta(vector_indices);
        m_a = m_a + del_m_a;
    end
    if do_bias
        del_bias_mag = delta(bias_indices);
        bias_mag = bias_mag + del_bias_mag;
    end
    if do_skew
         del_skew_mag = delta(skew_indices);
         skew_mag = skew_mag + del_skew_mag;
    end
    iter = iter + 1;
end

% compute error vector
e = errorMag(C_ms_mag, m_a, bias_mag, skew_mag, data_synced);
RMSE = sqrt((e.'*e)./numel(e(e~=0)));
disp(['RMSE After Calibration (uT): ' , num2str(RMSE)])

%% Results and Output
results.C_ms_mag = C_ms_mag;
results.m_a = m_a;
results.bias_mag = bias_mag;
results.scale_mag = scale_mag;
results.skew_mag = skew_mag;

% Corrected Measurements
T_skew = eye(3);
T_skew(1,2) = -skew_mag(3);
T_skew(1,3) =  skew_mag(2);
T_skew(2,3) = -skew_mag(1);
data_calibrated = data_synced;
data_calibrated.mag = C_ms_mag*T_skew*diag(scale_mag)*(data_synced.mag + results.bias_mag);

plotScript(results, data_calibrated, cost_history)

if is_si_units
    data_calibrated.mag = (1e-6)*data_calibrated.mag;
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function err = errorMag(C_ms_mag, m_a, bias, skew, data_synced)
    scale = [1;1;1];
    err = zeros(3,length(data_synced.t));
    is_static = data_synced.staticIndices;
    is_gap = data_synced.gapIndices;
    T_skew = eye(3);
    T_skew(1,2) = -skew(3);
    T_skew(1,3) =  skew(2);
    T_skew(2,3) = -skew(1);
    mag_corrected = C_ms_mag*T_skew*diag(scale)*(data_synced.mag + bias);
    for lv1=1:1:length(data_synced.t)
        err(:,lv1) = (m_a - data_synced.C_ba(:,:,lv1).'*mag_corrected(:,lv1));
    end
    %err(3,:) = 0.01*err(3,:);
    err(:,is_static) = 0;
    err(:,is_gap) = 0;
    err = err(:);
end


%% OPTIONS PROCESSING
function [tol, do_frame, do_bias, do_scale, do_skew, do_mag_vector]...
                                                       = processOptions(options)
if isfield(options,'tolerance')
    tol = options.tolerance;
else
    tol = 1e-8;
end

if isfield(options,'frames')
    do_frame = options.frames;
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
    do_scale = false;
end

if isfield(options,'skew')
    do_skew = options.skew;
else
    do_skew = false;
end

if isfield(options,'vector')
    do_mag_vector = options.vector;
else
    do_mag_vector = true;
end

end
%% IMPORT RESULTS PROCESSING
function [m_a, C_ms, bias_mag, scale_mag, skew_mag] ...
                                         =  processImportResults(import_results)
if isfield(import_results,'C_ms_mag')
    C_ms = import_results.C_ms_mag;
else
    C_ms = eye(3);
end
if isfield(import_results,'bias_mag')
    bias_mag = import_results.bias_mag;
else
    bias_mag = [0;0;0];
end
if isfield(import_results,'scale_mag')
    scale_mag = import_results.scale_mag;
else
    scale_mag = [1;1;1];
end
if isfield(import_results,'skew_mag')
    skew_mag = import_results.skew_mag;
else
    skew_mag = [0;0;0];
end

if isfield(import_results,'m_a')
    m_a = import_results.m_a;
else
    m_a = [0;0;0];
end

end

function plotScript(results, data_calibrated, cost_history)
m_b_predicted = zeros(3,length(data_calibrated.t));
for lv1=1:1:length(data_calibrated.t)
    m_b_predicted(:,lv1) = data_calibrated.C_ba(:,:,lv1)*results.m_a;
end

y_mag_a = zeros(3,length(data_calibrated.t));
for lv1=1:1:length(data_calibrated.t)
    y_mag_a(:,lv1) = data_calibrated.C_ba(:,:,lv1).'*data_calibrated.mag(:,lv1);
end

% Plotting to evaluate performance visually
figure
title('Magnetometer Calibration')
subplot(3,2,1)
plot(data_calibrated.t, m_b_predicted(1,:))
hold on
plot(data_calibrated.t, data_calibrated.mag(1,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_{b_x}$ [$\mu T$]', 'Interpreter', 'Latex','fontsize',14)
title('Body Frame')


subplot(3,2,3)
plot(data_calibrated.t, m_b_predicted(2,:))
hold on
plot(data_calibrated.t, data_calibrated.mag(2,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_{b_y}$ [$\mu T$]', 'Interpreter', 'Latex')

subplot(3,2,5)
plot(data_calibrated.t, m_b_predicted(3,:))
hold on
plot(data_calibrated.t, data_calibrated.mag(3,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_{b_z}$ [$\mu T$]', 'Interpreter', 'Latex')
legend('Calibrated $\mathbf{m}_b$', 'Calibrated Magnetometer $\mathbf{y}^{\mathrm{mag}}_b$',...
    'Interpreter', 'Latex','location','southoutside')

%%%%
subplot(3,2,2)
plot([data_calibrated.t(1), data_calibrated.t(end)], [results.m_a(1),results.m_a(1)])
hold on
plot(data_calibrated.t, y_mag_a(1,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_{a_x}$ [$\mu T$]', 'Interpreter', 'Latex','fontsize',14)
title('Local Mocap Frame')

subplot(3,2,4)
plot([data_calibrated.t(1), data_calibrated.t(end)], [results.m_a(2),results.m_a(2)])
hold on
plot(data_calibrated.t, y_mag_a(2,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_{a_y}$ [$\mu T$]', 'Interpreter', 'Latex','fontsize',14)

subplot(3,2,6)
plot([data_calibrated.t(1), data_calibrated.t(end)], [results.m_a(3),results.m_a(3)])
hold on
plot(data_calibrated.t, y_mag_a(3,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_{a_z}$ [$\mu T$]', 'Interpreter', 'Latex','fontsize',14)
legend('Calibrated $\mathbf{m}_a$', 'Calibrated Magnetometer $\mathbf{y}^{\mathrm{mag}}_a$',...
    'Interpreter', 'Latex','location','southoutside')

% Plot evolution of cost function
figure
plot(cost_history,'linewidth',2)
grid on
xlabel('Iteration Number', 'Interpreter', 'Latex')
ylabel('$J$ [$\left(m/s^2\right)^2$]', 'Interpreter', 'Latex')

end
