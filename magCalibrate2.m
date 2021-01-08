function [results, data_calibrated] = magCalibrate2(data_synced, options, import_results)
%MAGCALIBRATE2 Calibrates magnetometer.
%
% Example uses:
%       results = magCalibrate2(data_synced)
%       results = magCalibrate2(data_synced, options)
%       results = magCalibrate2(data_synced, options, import_results)
%       results = magCalibrate2(data_synced, [], import_results)
%       [results, data_calibrated] = magCalibrate2(data_synced)
%
% PARAMETERS:
% -----------
% data_synced: struct with fields (output of imuMocapSync())
%    t: [N x 1] double
%       imu timestamps in the mocap clock reference
%    mag: [3 x N] double
%       accelerometer raw data
%    r_zw_a: [3 x N] double
%       ground truth position of IMU in mocap local frame
%    C_ba: [3 x 3 x N] double
%       ground truth attitude of mocap body frame.
% options: struct with fields
%    distortion: boolean
%       set to true to calibrate distortion matrix (soft-iron effects).
%    bias: boolean
%       set to true to calibrate body-frame bias (hard-iron effects).
%    vector: boolean
%       set to true to calibrate local magnetic field vector
% import_results: struct with fields
%    D: [3 x 3] double
%       distortion matrix
%    bias: [3 x 1] double
%       magnetometer bias
%    m_a: [3 x 1] double
%       magnetic field vector in mocap local/world frame
%
% RETURNS:
% --------
% results: struct with fields
%    D: [3 x 3] double
%       distortion matrix
%    bias: [3 x 1] double
%       magnetometer bias
%    m_a: [3 x 1] double
%       magnetic field vector in mocap local/world frame

if ~exist('options','var')
    options = [];
end

% Normalize measurements for numerical conditioning.
% TODO. Need to figure this out properly.
normalization_factor = mean(vecnorm(data_synced.mag));
data_synced.mag = data_synced.mag./normalization_factor;

if nargin == 3
    D_0 = import_results.D;
    bias_0 = import_results.bias;
    m_a_0 = import_results.m_a;
else
    % INITIAL GUESS - Ellipsoid fitting
    % Kok, M., Hol, J. D., Schön, T. B., Gustafsson, F., & Luinge, H. (2012). 
    % Calibration of a magnetometer in combination with inertial sensors. 
    % 15th International Conference on Information Fusion, FUSION 2012, 787–793.

    mag = data_synced.mag;
    M = zeros(numel(data_synced.t), 13);
    for lv1 = 1:numel(data_synced.t)    
        M(lv1,:) = [kron(mag(:,lv1).', mag(:,lv1).'), mag(:,lv1).', 1];
    end
    [~,S,V] = svd(M,'econ');
    [~, col_number] = min(S(abs(S)>1e-10));
    eta = V(:,col_number);

    A_s = reshape(eta(1:9),3,3);
    b_s = eta(10:12);
    c_s = eta(13);

    alp = (0.25*b_s.'*A_s*b_s - c_s)^(-1);
    [D_0, is_failed] = chol(alp*A_s);
    if is_failed
        warning(sprintf(['\n',...
            'DECAR_MOCAP_TOOLS: Initial guess has failed.\n',...
            'DECAR_MOCAP_TOOLS: This is typically because the data does not contain\n',...
            'DECAR_MOCAP_TOOLS: a wide enough range of attitudes. We will attempt to\n',...
            'DECAR_MOCAP_TOOLS: optimize with a poor initial guess.\n'...
            ]));
        bias_0 = [0;0;0];
        D_0 = eye(3);
    else
        bias_0 = -0.5*A_s\b_s;
    end
    % Initial guess for m_a
    is_static = data_synced.staticIndices;
    m_a_data = zeros(3, numel(data_synced.t));
    for lv1=1:1:length(data_synced.t)
        m_a_data(:,lv1) = data_synced.C_ba(:,:,lv1).'*data_synced.mag(:,lv1);
    end
    m_a_0 = mean(m_a_data(:,~is_static),2,'omitnan');
    m_a_0 = m_a_0./norm(m_a_0);
end

% Convert m_a to an equivalent DCM. 
m_e = [0;0;1];
phi_ae_0 = acos(m_a_0.'*m_e);
axis_ae_0 = crossOp(m_e)*m_a_0;
axis_ae_0 = axis_ae_0./norm(axis_ae_0);
phi_vec_ae_0 = phi_ae_0*axis_ae_0;
if phi_vec_ae_0(3) ~= 0
    error('Program error!')
end
C_ae_0 = expm(crossOp(phi_vec_ae_0));

% REFINEMENT STEP 1 - Solving for only a rotation and mag vector
variables(1).x_0 = C_ae_0;
variables(1).update_func = @(X, dx) expm(crossOp([dx;0]))*X;
variables(1).dimension = 2;
variables(1).disabled = checkField(options, 'vector');

variables(2).x_0 = eye(3);
variables(2).update_func = @(X, dx) X*expm(crossOp(dx));
variables(2).dimension = 3;
variables(2).disabled = checkField(options, 'distortion');

err_func = @(x) errorMag1(x, D_0, bias_0, data_synced);
x_opt = leastSquares(err_func, variables);

C_ae_0 = x_opt{1};
C_0 = x_opt{2};

% REFINEMENT STEP 2 - Full Least-Squares Procedure
% C_ae
variables(1).x_0 = C_ae_0;
variables(1).update_func = @(X, dx) X*expm(crossOp([dx;0]));
variables(1).dimension = 2;
variables(1).disabled = checkField(options, 'vector');

% Distortion matrix D
variables(2).x_0 = D_0*C_0;
variables(2).update_func = @(X, dx) X + reshape(dx,3,3);
variables(2).dimension = 9;
variables(2).disabled = checkField(options, 'distortion');

% Bias
variables(3).x_0 = bias_0; 
variables(2).disabled = checkField(options, 'bias');

% Solve
err_func = @(x) errorMag2(x, data_synced);
x_opt = leastSquares(err_func, variables);

% FINAL RESULTS
results.m_a = x_opt{1}*[0;0;1];
results.D = x_opt{2};
results.bias = x_opt{3};

data_calibrated = data_synced;
data_calibrated.mag = results.D\(data_calibrated.mag - results.bias);
plotScript(results, data_calibrated);


end

function err = errorMag1(x, D, bias, data_synced)
    C_ae = x{1};
    C = x{2};

    err = zeros(3,length(data_synced.t));
    is_static = data_synced.staticIndices;
    is_gap = data_synced.gapIndices;
   
    m_a = C_ae*[0;0;1];

    for lv1=1:5:length(data_synced.t) % Downsample for speed.
        err(:,lv1) = data_synced.mag(:,lv1) - (D*C*data_synced.C_ba(:,:,lv1)*m_a + bias);
    end

    err(3,:) = err(3,:);
    err(:,is_static) = 0;
    err(:,is_gap) = 0;
    err = err(:);
end

function err = errorMag2(x, data_synced)
    C_ae = x{1};
    D = x{2};
    D = reshape(D,3,3);
    bias = x{3};
    err = zeros(3,length(data_synced.t));
    is_static = data_synced.staticIndices;
    is_gap = data_synced.gapIndices;
   
    m_a = C_ae*[0;0;1];

    for lv1=1:1:length(data_synced.t)
        err(:,lv1) = data_synced.mag(:,lv1) - (D*data_synced.C_ba(:,:,lv1)*m_a + bias);
    end

    err(3,:) = err(3,:);
    err(:,is_static) = 0;
    err(:,is_gap) = 0;
    err = err(:);
end

function tf = checkField(S, varargin)
    tf = false;
    if isa(S,'struct')
        for lv1 = 1:numel(varargin)
            fieldname = varargin{lv1};
            if isfield(S, fieldname)
                tf = S.(fieldname);
                break;
            end
        end
    end
end

function plotScript(results, data_calibrated)
    m_a = results.m_a;
    y_mag_a = zeros(3,numel(data_calibrated.t));
    for lv1 = 1:numel(data_calibrated.t)
        y_mag_a(:,lv1) = data_calibrated.C_ba(:,:,lv1).'*data_calibrated.mag(:,lv1);
    end

    y_hat_b = zeros(3,numel(data_calibrated.t));
    for lv1 = 1:numel(data_calibrated.t)
        y_hat_b(:,lv1) = data_calibrated.C_ba(:,:,lv1)*m_a;
    end


    figure
    subplot(3,1,1)
    plot(data_calibrated.t, y_mag_a(1,:),'linewidth',2)
    hold on
    plot([data_calibrated.t(1), data_calibrated.t(end)], [m_a(1), m_a(1)],'linewidth',2)
    hold off
    grid on
    axis([-inf inf -1 1])
    ylabel('$m_{a_x}$','interpreter','latex','fontsize',14)
    title('Local frame')
    legend('$\mathbf{C}_{ab}\mathbf{y}_b^{\mathrm{mag}}$','$\mathbf{m}_a$','interpreter','latex','fontsize',11)

    subplot(3,1,2)
    plot(data_calibrated.t, y_mag_a(2,:),'linewidth',2)
    hold on
    plot([data_calibrated.t(1), data_calibrated.t(end)], [m_a(2), m_a(2)],'linewidth',2)
    hold off
    grid on
    axis([-inf inf -1 1])
    ylabel('$m_{a_y}$','interpreter','latex','fontsize',14)
    
    subplot(3,1,3)
    plot(data_calibrated.t, y_mag_a(3,:),'linewidth',2)
    hold on
    plot([data_calibrated.t(1), data_calibrated.t(end)], [m_a(3), m_a(3)],'linewidth',2)
    hold off
    grid on
    axis([-inf inf -1 1])
    ylabel('$m_{a_z}$','interpreter','latex','fontsize',14)
    xlabel('Time (s)','interpreter','latex','fontsize',14)
    
    figure 
    subplot(3,1,1)
    plot(data_calibrated.t, data_calibrated.mag(1,:),'linewidth',2)
    hold on
    plot(data_calibrated.t, y_hat_b(1,:),'linewidth',2)
    hold off
    grid on
    axis([-inf inf -1 1])
    title('Body frame')
    legend('Magnetometer measurement','$\mathbf{m}_b$','interpreter','latex','fontsize',11)
    ylabel('$m_{a_x}$','interpreter','latex','fontsize',14)
    
    subplot(3,1,2)
    plot(data_calibrated.t, data_calibrated.mag(2,:),'linewidth',2)
    hold on
    plot(data_calibrated.t, y_hat_b(2,:),'linewidth',2)
    hold off
    grid on
    axis([-inf inf -1 1])
    ylabel('$m_{a_y}$','interpreter','latex','fontsize',14)
    
    subplot(3,1,3)
    plot(data_calibrated.t, data_calibrated.mag(3,:),'linewidth',2)
    hold on
    plot(data_calibrated.t, y_hat_b(3,:),'linewidth',2)
    hold off
    grid on
    axis([-inf inf -1 1])
    ylabel('$m_{a_z}$','interpreter','latex','fontsize',14)
    xlabel('Time (s)','interpreter','latex','fontsize',14)
end