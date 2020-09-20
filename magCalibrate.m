function [results, dataCalibrated, RMSE] = magCalibrate(data_synced, options, import_results)
% Find the DCM C_sm representing the rotation between the sensor frame of the
% IMU (F_s) and the body frame assigned by the Mocap system (F_m), as well
% as the accelerometer and gyroscope biases.
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


[tol, do_frame, do_bias, do_scale, do_skew, do_mag_vector] = processOptions(options);
[m_a, C_ms_mag, bias_mag, scale_mag, skew_mag] =  processImportResults(import_results);

%% LS Optimization on SO(3), using Gauss-Newton

% initialize
costFuncHist = [];
%phi = [0;0;0];
%m_a = [0;0;0];
%bias_mag = [0;0;0];
%scale_mag = [1;1;1];
%C_ms_mag = ROTVEC_TO_DCM(phi);

delta = Inf;
iter = 0;
while norm(delta) > tol && iter < 100
    % compute error vector
    e = errorMag(C_ms_mag, m_a, bias_mag, scale_mag, data_synced);
    
    % compute Jacobians
    A = [];
    indx_counter = 1;
    if do_frame
        f_C_ms_mag = @(C) errorMag(C, m_a, bias_mag, scale_mag, data_synced);
        A_phi_mag = complexStepJacobianLie(f_C_ms_mag,C_ms_mag,...
            3,@CrossOperator,'direction','right');
        A = [A, A_phi_mag];
        phi_indices = indx_counter:indx_counter+2;
        indx_counter = indx_counter + 3;
    end
    
    if do_mag_vector
        f_m_a = @(m) errorMag(C_ms_mag, m, bias_mag, scale_mag, data_synced);
        A_m_a = complexStepJacobian(f_m_a, m_a);
        A = [A, A_m_a];
        vector_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_bias
        f_bias = @(b) errorMag(C_ms_mag, m_a, b, scale_mag, data_synced);
        A_bias = complexStepJacobian(f_bias, bias_mag);
        A = [A, A_bias];
        bias_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    if do_scale
        f_scale = @(scale) errorMag(C_ms_mag, m_a, bias_mag, scale, data_synced);
        A_scale = complexStepJacobian(f_scale, scale_mag);
        A = [A, A_scale];
        scale_indices = indx_counter:indx_counter + 2;
        indx_counter = indx_counter + 3;
    end
    
    cost = 0.5*(e.'*e)
    costFuncHist = [costFuncHist; cost];
    
    % Compute step direction
    if iter > 20
        delta = -(A.'*A + 0.2*diag(diag(A.'*A))) \ (A.' * e);
    else
        delta = -(A.'*A) \( A.' * e);
    end
    
    % decompose and update
    if do_frame
        del_phi_mag    = delta(phi_indices);
        C_ms_mag = C_ms_mag*ROTVEC_TO_DCM(-del_phi_mag);
    end
    if do_mag_vector
        del_m_a = delta(vector_indices);
        m_a = m_a + del_m_a;
    end
    if do_bias
        del_bias_mag = delta(bias_indices);
        bias_mag = bias_mag + del_bias_mag;
    end
    if do_scale
    %     del_scale_mag = delta(scale_indices);
    %     scale_mag = scale_mag + del_scale_mag;
    end
    iter = iter + 1;
end

% compute error vector
e = errorMag(C_ms_mag, m_a, bias_mag, scale_mag, data_synced);
RMSE = sqrt((e.'*e)./length(data_synced.t));
disp(['RMSE After Calibration: ' , num2str(RMSE)])

%% Results and Output
results.C_ms_mag = C_ms_mag;
results.m_a = m_a;
results.bias_mag = bias_mag;
results.scale_mag = scale_mag;

% Compute calibrated m_b.
m_b_predicted = zeros(3,length(data_synced.mag));
for lv1=1:1:length(data_synced.mag)
    m_b_predicted(:,lv1) = diag(scale_mag) \ C_ms_mag' * data_synced.C_ba(:,:,lv1) * results.m_a - results.bias_mag;
end

dataCalibrated = data_synced;
dataCalibrated.mag = C_ms_mag*diag(scale_mag)*(data_synced.mag + results.bias_mag);

%% Plotting to evaluate performance visually
figure
subplot(3,1,1)
plot(data_synced.t, m_b_predicted(1,:))
hold on
plot(data_synced.t, data_synced.mag(1,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_b^1$ [$\mu T$]', 'Interpreter', 'Latex')
legend('Calibrated $\mathbf{m}_b$', 'Magnetometer measurement',...
    'Interpreter', 'Latex')

subplot(3,1,2)
plot(data_synced.t, m_b_predicted(2,:))
hold on
plot(data_synced.t, data_synced.mag(2,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_b^2$ [$\mu T$]', 'Interpreter', 'Latex')

subplot(3,1,3)
plot(data_synced.t, m_b_predicted(3,:))
hold on
plot(data_synced.t, data_synced.mag(3,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_b^3$ [$\mu T$]', 'Interpreter', 'Latex')

% Plot evolution of cost function
figure
plot(costFuncHist)
grid
xlabel('Iteration Number', 'Interpreter', 'Latex')
ylabel('$J$ [$\left(m/s^2\right)^2$]', 'Interpreter', 'Latex')

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function err = errorMag(C_ms_mag, m_a, bias, scale, data_synced)
err = zeros(3,length(data_synced.t));
is_static = data_synced.staticIndices;
mag_corrected = C_ms_mag*diag(scale)*(data_synced.mag + bias);
for lv1=1:1:length(data_synced.t)
    %         if data_synced.staticIndices(lv1)
    % %             cov_mag = cov(data_synced.mag(:,lv1).');
    %             W_mag = 0;%inv(diag(diag(chol(cov_mag))));
    %         else
    %             W_mag = 1;%eye(3);
    %         end
    err(:,lv1) = (m_a - data_synced.C_ba(:,:,lv1).'*mag_corrected(:,lv1));
end
err(:,is_static) = 0;
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
    m_a = [0; 0; 0];
end

end

