function [results, dataCalibrated, RMSE] = magCalibrate(data_synced, options)
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
    TOL = 1e-4;
end

%% LS Optimization on SO(3), using Gauss-Newton

% initialize
costFuncHist = [];
phi = [0;0;0];
m_a = [0;0;0];
bias_mag = [0;0;0];
scale_mag = [1;1;1];
C_ms_mag = ROTVEC_TO_DCM(phi);

delta = Inf;
iter = 0;
while norm(delta) > TOL && iter < 100
    % compute error vector
    e = errorMag(C_ms_mag, m_a, bias_mag, scale_mag, data_synced);
    
    % compute Jacobians    
    f_C_ms_mag = @(C) errorMag(C, m_a, bias_mag, scale_mag, data_synced);
    A_phi_mag = complexStepJacobianLie(f_C_ms_mag,C_ms_mag,3,@CrossOperator,'direction','right');
    
    f_m_a = @(m) errorMag(C_ms_mag, m, bias_mag, scale_mag, data_synced);
    A_m_a = complexStepJacobian(f_m_a, m_a);

    f_bias = @(b) errorMag(C_ms_mag, m_a, b, scale_mag, data_synced);
    A_bias = complexStepJacobian(f_bias, bias_mag);
    
%     f_scale = @(scale) errorMag(C_ms_mag, m_a, bias_mag, scale, data_synced);
%     A_scale = complexStepJacobian(f_scale, scale_mag);
    
%     A = [A_phi_mag, A_m_a, A_bias, A_scale];
    A = [A_phi_mag, A_m_a, A_bias];
    
    cost = 0.5*(e.'*e)
    costFuncHist = [costFuncHist; cost];
    
    % Compute step direction
    if iter > 20
        delta = -(A.'*A + 0.2*diag(diag(A.'*A))) \ (A.' * e);
    else
        delta = -(A.'*A) \( A.' * e);
    end
    
    % decompose and update
    del_phi_mag    = delta(1:3);
    C_ms_mag = C_ms_mag*ROTVEC_TO_DCM(-del_phi_mag);
    
    del_m_a = delta(4:6);
    m_a = m_a + del_m_a;
    
    del_biasMag_s = delta(7:9);
    bias_mag = bias_mag + del_biasMag_s;
    
%     del_scale_mag = delta(10:12);
%     scale_mag = scale_mag + del_scale_mag;

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

dataCalibrated = data_synced;
% dataCalibrated.mag = C_ms_mag*(dataCalibrated.mag+ bias_mag);

% Compute calibrated m_b.
m_b_calibrated = zeros(3,length(data_synced.mag));
for lv1=1:1:length(data_synced.mag)
    m_b_calibrated(:,lv1) = diag(scale_mag) \ C_ms_mag' * data_synced.C_ba(:,:,lv1) * results.m_a - results.bias_mag;
end

%% Plotting to evaluate performance visually
figure
subplot(3,1,1)
plot(data_synced.t, m_b_calibrated(1,:))
hold on
plot(data_synced.t, data_synced.mag(1,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_b^1$ [$\mu T$]', 'Interpreter', 'Latex')
legend('Calibrated $\mathbf{m}_b$', 'Magnetometer measurement',...
       'Interpreter', 'Latex')

subplot(3,1,2)
plot(data_synced.t, m_b_calibrated(2,:))
hold on
plot(data_synced.t, data_synced.mag(2,:))
hold off
grid on
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$m_b^2$ [$\mu T$]', 'Interpreter', 'Latex')

subplot(3,1,3)
plot(data_synced.t, m_b_calibrated(3,:))
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
    err = [];
    for lv1=1:1:length(data_synced.t)
        if data_synced.staticIndices(lv1)
%             cov_mag = cov(data_synced.mag(:,lv1).');
            W_mag = 1;%inv(diag(diag(chol(cov_mag))));
        else
            W_mag = 1;%eye(3);
        end
        err_iter = W_mag*(m_a - data_synced.C_ba(:,:,lv1).'*C_ms_mag*diag(scale)*(data_synced.mag(:,lv1) + bias));
        err = [err; err_iter];
    end
end
