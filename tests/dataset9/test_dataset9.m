% An example of how to use the Mocap tools.

clear; close all;
%% Get IMU position
r_imuz_b = mocap_getPointInBodyFrame('2020_08_04_180_calibration_trial7',...
                                     'RigidBody','Unlabeled4738')
%% Extract Mocap data
dataMocap = mocap_csv2struct('2020_08_04_180_mocap_trial9.csv')

%% Align the reference point and the IMU
% TODO: 1) include in mocap_csv2struct?
for lv1=1:1:length(dataMocap.RigidBody.t)
    dataMocap.RigidBody.r_zw_a(:,lv1) = dataMocap.RigidBody.r_zw_a(:,lv1) + ...
                                           dataMocap.RigidBody.C_ba(:,:,lv1).'*r_imuz_b;
end

%% Fit a b-spline to the Mocap data
splineMocap = mocap_fitSpline(dataMocap,[],true)

%% Extract the IMU data
dataIMU = IMU_csv2struct('2020_07_30_23_29_08_trial9_sensehat_180.csv')

%% Synchronize the Mocap and IMU data
tic
dataSynced = syncTime(splineMocap.RigidBody, dataIMU)
toc
%% Disregard IMU data within the time ranges where no ground truth was collected
[dataSyncedCleaned, gapIndices] = deleteGaps(dataSynced, dataMocap.RigidBody.mocapGaps)
dataSynced.gapIndices = gapIndices;

%% Align the frames of the Mocap and IMU data to find an initial DCM
tic
dataAligned = alignFrames(dataSynced)
toc
%% Refine the DCM between the two assigned body frames

[C_ms, biasAcc, biasGyr, dataCalibrated] = calibrateFrames(dataAligned)

%% Dead Reckon Spline to Validate 
t = (dataCalibrated.t(1):0.0001:dataCalibrated.t(end)).';
temp      = ppval(splineMocap.RigidBody,t);
tempDerv  = splineDerv(splineMocap.RigidBody, t, 1);
tempDerv2 = splineDerv(splineMocap.RigidBody, t, 2);

% Generate the data first
accMocap   = zeros(3,length(t));
omegaMocap = zeros(3,length(t));
g_a = [0;0;-9.792];
for lv1=1:length(t)

    % Mocap omega data
    q_ba     = temp(4:7,lv1);
    q_ba = q_ba./norm(q_ba);
    q_ba_dot = tempDerv(4:7,lv1);
    omega_ba_b = quatrate2omega(q_ba, q_ba_dot);
    omegaMocap(:,lv1) = omega_ba_b;

    % Mocap acceleration data
    C_ba = quat2dcm(q_ba.');
    accMocap(:,lv1) = C_ba*(tempDerv2(1:3,lv1) - g_a);
end

N = length(t);
r_zw_a = zeros(3,N);
v_zwa_a = zeros(3,N);
C_ba = zeros(3,3,N);
r_zw_a(:,1) = dataMocap.RigidBody.r_zw_a(:,1);
C_ba(:,:,1) = dataMocap.RigidBody.C_ba(:,:,1);
g_a = [0;0;-9.792];
for lv1 = 1:N-1
    dt = t(lv1+1) - t(lv1);
    omega_ba_b = omegaMocap(:,lv1);
    a_zwa_b = accMocap(:,lv1);  
    C_ba(:,:,lv1+1) = expm(-CrossOperator(omega_ba_b*dt))*C_ba(:,:,lv1);
    v_zwa_a(:,lv1+1) = v_zwa_a(:,lv1) + (C_ba(:,:,lv1).'*a_zwa_b + g_a)*dt;
    r_zw_a(:,lv1+1) = r_zw_a(:,lv1) + v_zwa_a(:,lv1)*dt;
end

figure
plot3(r_zw_a(1,1:end), r_zw_a(2,1:end), r_zw_a(3,1:end),'linewidth',2)
hold on
plot3(dataMocap.RigidBody.r_zw_a(1,1:end),...
      dataMocap.RigidBody.r_zw_a(2,1:end),...
      dataMocap.RigidBody.r_zw_a(3,1:end),'linewidth',2);
hold off
axis vis3d
axis equal
legend('Dead-Reckon Solution','Ground Truth')
title('Ground Truth Dead-Reckoning')
%axis([-1 4 -2 2 0 3])
xlabel('x')
ylabel('y')
zlabel('z')
grid on


%% Dead Reckon Actual Data to Test
N = length(dataCalibrated.t);
r_zw_a = zeros(3,N);
v_zwa_a = zeros(3,N);
C_ba = zeros(3,3,N);
r_zw_a(:,1) = dataMocap.RigidBody.r_zw_a(:,1);
C_ba(:,:,1) = dataMocap.RigidBody.C_ba(:,:,1);
g_a = [0;0;-9.792];
for lv1 = 1:N-1
    dt = dataCalibrated.t(lv1+1) - dataCalibrated.t(lv1);
    if (dataCalibrated.t(lv1) > 19) && (dataCalibrated.t(lv1) < 140)
        omega_ba_b = dataCalibrated.omegaIMU(:,lv1);
        a_zwa_b = dataCalibrated.accIMU(:,lv1);
    else
        omega_ba_b = dataCalibrated.omegaMocap(:,lv1);
        a_zwa_b = dataCalibrated.accMocap(:,lv1);  
    end
    C_ba(:,:,lv1+1) = expm(-CrossOperator(omega_ba_b*dt))*C_ba(:,:,lv1);
    v_zwa_a(:,lv1+1) = v_zwa_a(:,lv1) + (C_ba(:,:,lv1).'*a_zwa_b + g_a)*dt;
    r_zw_a(:,lv1+1) = r_zw_a(:,lv1) + v_zwa_a(:,lv1)*dt;
    
end
phi_ba = DCM_TO_ROTVEC(C_ba);
phi_ba_mocap = DCM_TO_ROTVEC(dataMocap.RigidBody.C_ba);
figure
plot3(r_zw_a(1,1:6000), r_zw_a(2,1:6000), r_zw_a(3,1:6000),'linewidth',2)
hold on
plot3(dataMocap.RigidBody.r_zw_a(1,1:3175),...
      dataMocap.RigidBody.r_zw_a(2,1:3175),...
      dataMocap.RigidBody.r_zw_a(3,1:3175),'linewidth',2);
hold off
axis vis3d
axis equal
legend('Dead-Reckon Solution','Ground Truth')
title('Position')
%axis([-1 4 -2 2 0 3])
xlabel('x')
ylabel('y')
zlabel('z')
grid on

figure
subplot(3,1,1)
plot(dataCalibrated.t, phi_ba(1,:))
hold on
plot(dataMocap.RigidBody.t, phi_ba_mocap(1,:))
hold off
grid on
ylabel('$\phi_1$','interpreter','latex')

subplot(3,1,2)
plot(dataCalibrated.t, phi_ba(2,:))
hold on
plot(dataMocap.RigidBody.t, phi_ba_mocap(2,:))
hold off
grid on
ylabel('$\phi_2$','interpreter','latex')

subplot(3,1,3)
plot(dataCalibrated.t, phi_ba(3,:))
hold on
plot(dataMocap.RigidBody.t, phi_ba_mocap(3,:))
hold off
grid on
ylabel('$\phi_3$','interpreter','latex')