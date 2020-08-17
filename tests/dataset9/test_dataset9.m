% An example of how to use the Mocap tools.
clear;
close all;
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
%% Obtain indices of gaps in mocap data, and identified stationary periods
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
%% Align the frames of the Mocap and IMU data to find an initial DCM
tic
dataAligned = alignFrames(dataSynced)
toc
%% Refine the DCM between the two assigned body frames
options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
[results, dataCalibrated] = calibrateFrames(dataAligned, options)

%% Dead Reckon Spline to Validate 
t = (dataCalibrated.t(1):0.001:dataCalibrated.t(end)).';

% Generate the data first
g_a = results.g_a;
[accMocap, omegaMocap] = getFakeImuMocap(splineMocap.RigidBody,t,g_a);

N = length(t);
r_zw_a = zeros(3,N);
v_zwa_a = zeros(3,N);
a_zwa_a = zeros(3,N);
C_ba = zeros(3,3,N);
r_zw_a(:,1) = dataMocap.RigidBody.r_zw_a(:,1);
C_ba(:,:,1) = dataMocap.RigidBody.C_ba(:,:,1);

for lv1 = 1:N-1
    dt = t(lv1+1) - t(lv1);
    omega_ba_b = omegaMocap(:,lv1);
    a_zwa_b = accMocap(:,lv1);  
    C_ba(:,:,lv1+1) = expm(-CrossOperator(omega_ba_b*dt))*C_ba(:,:,lv1);
    v_zwa_a(:,lv1+1) = v_zwa_a(:,lv1) + (C_ba(:,:,lv1).'*a_zwa_b + g_a)*dt;
    r_zw_a(:,lv1+1) = r_zw_a(:,lv1) + v_zwa_a(:,lv1)*dt;
    a_zw_a(:,lv1) = C_ba(:,:,lv1).'*a_zwa_b + g_a;
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
g_a = results.g_a;
a_zwa_a = zeros(3,N);
for lv1 = 1:N-1
    dt = dataCalibrated.t(lv1+1) - dataCalibrated.t(lv1);
    if (dataCalibrated.t(lv1) > 0) && (dataCalibrated.t(lv1) < 140)
        omega_ba_b = dataCalibrated.omegaIMU(:,lv1);
        a_zwa_b = dataCalibrated.accIMU(:,lv1);
    else
        omega_ba_b = dataCalibrated.omegaMocap(:,lv1);
        a_zwa_b = dataCalibrated.accMocap(:,lv1);  
    end
    C_ba(:,:,lv1+1) = expm(-CrossOperator(omega_ba_b*dt))*C_ba(:,:,lv1);
    v_zwa_a(:,lv1+1) = v_zwa_a(:,lv1) + (C_ba(:,:,lv1).'*a_zwa_b + g_a)*dt;
    r_zw_a(:,lv1+1) = r_zw_a(:,lv1) + v_zwa_a(:,lv1)*dt;
    a_zwa_a(:,lv1) = C_ba(:,:,lv1).'*a_zwa_b + g_a;
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
legend('Dead-Reckoning','Ground Truth')
title('Attitude Dead-reckoning Performance')

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

figure
subplot(3,1,1)
plot(dataCalibrated.t(1:end), a_zwa_a(1,:))
grid on

subplot(3,1,2)
plot(dataCalibrated.t(1:end), a_zwa_a(2,:))
grid on

subplot(3,1,3)
plot(dataCalibrated.t(1:end), a_zwa_a(3,:))
grid on


