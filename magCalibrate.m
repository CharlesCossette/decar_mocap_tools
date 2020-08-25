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
    TOL = 1e-10;
end

%% LS Optimization on SO(3), using Gauss-Newton

% initialize
costFuncHist = [];
m_a = [0;0;0];
bias_mag = [0;0;0];

delta = Inf;
iter = 0;
while norm(delta) > TOL && iter < 100
    % compute error vector
    e = errorMag(m_a, bias_mag, data_synced);
    
    % compute Jacobians    
    f_m_a = @(m) errorMag(m, bias_mag, data_synced);
    A_m_a = complexStepJacobian(f_m_a, m_a);

    f_bias = @(b) errorMag(m_a, b, data_synced);
    A_bias = complexStepJacobian(f_bias, bias_mag);
    
    A = [A_m_a, A_bias];
    
    cost = 0.5*(e.'*e)
    costFuncHist = [costFuncHist; cost];
    
    % Compute step direction
    if iter > 20
        delta = -(A.'*A + 0.2*diag(diag(A.'*A))) \ (A.' * e);
    else
        delta = -(A.'*A) \( A.' * e);
    end
    
    % decompose and update
    del_m_a = delta(1:3);
    m_a = m_a + del_m_a;
    
    del_biasMag_s = delta(4:6);
    bias_mag = bias_mag + del_biasMag_s;

    iter = iter + 1;
end

% compute error vector
e = errorMag(m_a, bias_mag, data_synced);
RMSE = sqrt((e.'*e)./length(data_synced.t));
disp(['RMSE After Calibration: ' , num2str(RMSE)])

%% Results and Output
results.m_a = m_a;
results.bias_mag = bias_mag;

dataCalibrated = data_synced;
% dataCalibrated.mag = C_ms_mag*(dataCalibrated.mag+ bias_mag);

% % Calibrated ground truth accel/gyro measurements.
% mocap_gyro = data_synced.gyro_mocap;
% mocap_accel = zeros(3, length(data_synced.t));
% for lv1 = 1:length(data_synced.t)
%     mocap_accel(:,lv1) = data_synced.accel_mocap(:,lv1) - data_synced.C_ba(:,:,lv1)*(g_a - g_e);
% end
% 
% dataCalibrated.t = data_synced.t;
% dataCalibrated.accel_mocap = mocap_accel;
% dataCalibrated.gyro_mocap = mocap_gyro;

% Compute calibrated m_b.
m_b_calibrated = zeros(3,length(data_synced.mag));
for lv1=1:1:length(data_synced.mag)
    m_b_calibrated(:,lv1) = data_synced.C_ba(:,:,lv1) * results.m_a - results.bias_mag;
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
function err = errorMag(m_a, bias, data_synced)
    err = [];
    for lv1=1:1:length(data_synced.t)
        err_iter = m_a - data_synced.C_ba(:,:,lv1).'*(data_synced.mag(:,lv1) + bias);
        err = [err; err_iter];
    end
end
