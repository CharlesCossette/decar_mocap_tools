function [data_aligned, C_ma, C_mg] = alignFrames(data_synced, rightHandedOnly)
% If rightHanded == true, then transformations are restricted to be
% positive-determinant DCMs.

if nargin < 2
    rightHandedOnly = false; % by default, assume the possibility of
    % left-handed frames
end

C_all = getAllPermutationMatrices(rightHandedOnly);

% We now have a set of all the possible DCMs corresponding to different
% axis switches, including left-handed frames.
% Loop through all the DCMs and try every single one. Find the DCMs
% with the lowest error.
% We evaluate accel and gyro completely seperately, as it is possible
% for them to be using different frames.
J_best_accel = inf;
J_best_gyro = inf;
C_ma_best = eye(3); % C_am is a dcm from mocap body frame to accel frame.
C_mg_best = eye(3); % C_gm is a dcm from mocap body frame to gyro frame.
for lv1 = 1:size(C_all,3)
    C = C_all(:,:,lv1);
    
    % Evaluate the accelerometer error, check if new best.
    e_accel = computeErrorAccel(C,data_synced);
    J_accel = 0.5*(e_accel.'*e_accel);
    if J_accel < J_best_accel
        C_ma_best = C;
        J_best_accel = J_accel;
    end
    
    % Evaluate the gyroscope error, check if new best.
    e_gyro = computeErrorGyro(C,data_synced);
    J_gyro = 0.5*(e_gyro.'*e_gyro);
    if J_gyro < J_best_gyro
        C_mg_best = C;
        J_best_gyro = J_gyro;
    end
end

if any(C_ma_best ~= C_mg_best,'all')
    warning('DECAR_MOCAP_TOOLS: Different accel/gyro frames detected!')
    warning('DECAR_MOCAP_TOOLS: Gyro will be transformed to the same frame as accel.')
    
end

if rightHandedOnly
    if abs(det(C_ma_best) - 1) > 1e-14 || abs(det(C_mg_best) - 1) > 1e-14 
        error('DECAR_MOCAP_TOOLS: Program error. DCM determinant is negative.')
    end
end

disp('Best fit accelerometer DCM:')
C_ma_best
disp('Best fit gyroscope DCM:')
C_mg_best

data_aligned = data_synced;
data_aligned.accel = C_ma_best*data_synced.accel;
data_aligned.gyro = C_mg_best*data_synced.gyro;
C_ma = C_ma_best;
C_mg = C_mg_best;

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

function output = computeErrorAccel(C_ms,dataSynced)
gaps = dataSynced.gapIndices;
error_accel = dataSynced.accel_mocap(:,~gaps) - C_ms*dataSynced.accel(:,~gaps);
output = error_accel(:);
end
function output = computeErrorGyro(C_ms,dataSynced)
gaps = dataSynced.gapIndices;
error_omega = dataSynced.gyro_mocap(:,~gaps) - C_ms*dataSynced.gyro(:,~gaps);
output = error_omega(:);
end