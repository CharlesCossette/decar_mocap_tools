function [data_aligned, C_ma, C_mg] = alignFrames(data_synced, right_handed_only)
% If rightHanded == true, then transformations are restricted to be
% positive-determinant DCMs.

if nargin < 2
    right_handed_only = false; % by default, assume the possibility of
    % left-handed frames
end

C_ma = findBestFrame(data_synced, @computeErrorAccel, right_handed_only);
C_mg = findBestFrame(data_synced, @computeErrorGyro, right_handed_only);

if any(C_ma ~= C_mg,'all')
    warning('DECAR_MOCAP_TOOLS: Different accel/gyro frames detected!')
    warning('DECAR_MOCAP_TOOLS: Gyro will be transformed to the same frame as accel.')
    
end

disp('Best fit accelerometer DCM:')
C_ma
disp('Best fit gyroscope DCM:')
C_mg

data_aligned = data_synced;
data_aligned.accel = C_ma*data_synced.accel;
data_aligned.gyro = C_mg*data_synced.gyro;

plotScript(data_aligned)
end
function C_ms_best = findBestFrame(data_synced, errorFunc, right_handed_only)
    C_all = getAllPermutationMatrices(right_handed_only);

    % We now have a set of all the possible DCMs corresponding to different
    % axis switches, including left-handed frames.
    % Loop through all the DCMs and try every single one. Find the DCMs
    % with the lowest error.
    % We evaluate accel and gyro completely seperately, as it is possible
    % for them to be using different frames.
    J_best = inf;
    C_ms_best = eye(3); % C_am is a dcm from mocap body frame to accel frame.
    for lv1 = 1:size(C_all,3)
        C = C_all(:,:,lv1);

        % Evaluate the accelerometer error, check if new best.
        e = errorFunc(C, data_synced);
        J = 0.5*(e.'*e);
        if J < J_best
            C_ms_best = C;
            J_best = J;
        end
    end

    if right_handed_only
        if abs(det(C_ms_best) - 1) > 1e-14 
            error('DECAR_MOCAP_TOOLS: Program error. DCM determinant is negative.')
        end
    end
end
function output = computeErrorAccel(C_ms,data_synced)
    gaps = data_synced.gapIndices;
    error_accel = data_synced.accel_mocap(:,~gaps) - C_ms*data_synced.accel(:,~gaps);
    output = error_accel(:);
end
function output = computeErrorGyro(C_ms,data_synced)
    gaps = data_synced.gapIndices;
    error_omega = data_synced.gyro_mocap(:,~gaps) - C_ms*data_synced.gyro(:,~gaps);
    output = error_omega(:);
end

function plotScript(data_aligned)
    %% Plotting to evaluate performance visually
    % Plot calibrated data - accelerometers
    figure
    sgtitle('Accelerometer')
    subplot(3,1,1)
    plot(data_aligned.t, data_aligned.accel(1,:))
    hold on
    plot(data_aligned.t, data_aligned.accel_mocap(1,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\ddot{x}$ [$m/s^2$]', 'Interpreter', 'Latex')
    legend('Calibrated IMU Data', 'Mocap Data')
    subplot(3,1,2)
    plot(data_aligned.t, data_aligned.accel(2,:))
    hold on
    plot(data_aligned.t, data_aligned.accel_mocap(2,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\ddot{y}$ [$m/s^2$]', 'Interpreter', 'Latex')
    subplot(3,1,3)
    plot(data_aligned.t, data_aligned.accel(3,:))
    hold on
    plot(data_aligned.t, data_aligned.accel_mocap(3,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\ddot{z}$ [$m/s^2$]', 'Interpreter', 'Latex')

    % Plot calibrated data - gyroscopes
    figure
    sgtitle('Gyroscope')
    subplot(3,1,1)
    plot(data_aligned.t, data_aligned.gyro(1,:))
    hold on
    plot(data_aligned.t, data_aligned.gyro_mocap(1,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\omega_x$ [rad/$s$]', 'Interpreter', 'Latex')
    legend('Calibrated IMU Data', 'Mocap Data')

    subplot(3,1,2)
    plot(data_aligned.t, data_aligned.gyro(2,:))
    hold on
    plot(data_aligned.t, data_aligned.gyro_mocap(2,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\omega_y$ [rad/$s$]', 'Interpreter', 'Latex')
    subplot(3,1,3)
    plot(data_aligned.t, data_aligned.gyro(3,:))
    hold on
    plot(data_aligned.t, data_aligned.gyro_mocap(3,:))
    hold off
    grid on
    xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
    ylabel('$\omega_z$ [rad/$s$]', 'Interpreter', 'Latex')
end