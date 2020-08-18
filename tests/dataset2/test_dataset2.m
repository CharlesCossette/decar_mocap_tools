% An example of how to use the Mocap tools.

clear; close all;
%% Get IMU position
r_imuz_b = mocap_getPointInBodyFrame('2020_08_03_155_imu_position_calibration.csv',...
                                     'RigidBody','Unlabeled3725')
%% Extract Mocap data
dataMocap = mocap_csv2struct('2020_08_03_155_mocap.csv')

%% Align the reference point and the IMU
% TODO: 1) include in mocap_csv2struct?
for lv1=1:1:length(dataMocap.RigidBody.t)
    dataMocap.RigidBody.r_zw_a(:,lv1) = dataMocap.RigidBody.r_zw_a(:,lv1) + ...
                                           dataMocap.RigidBody.C_ba(:,:,lv1).'*r_imuz_b;
end

%% Fit a b-spline to the Mocap data
spline = mocap_fitSpline(dataMocap,[],true)

%% Extract the IMU data
dataIMU = IMU_csv2struct('2020_07_28_23_53_14_sensehat_155.csv')

%% Synchronize the Mocap and IMU data
tic
dataSynced = syncTime(spline.RigidBody, dataIMU)
toc
%% Obtain indices of gaps in mocap data, and identified stationary periods
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);

%% Align the frames of the Mocap and IMU data to find an initial DCM
tic
dataAligned = alignFrames(dataSynced)
toc
%% Refine the DCM between the two assigned body frames
clear options
options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
options.start_index = 1;
options.max_total_states = 30000;
options.interval_size = 2000;
options.batch_size = 500;
[results, dataCalibrated] = calibrateImu(dataAligned,options)
%% Dead Reckon Actual Data to Test
r_zw_a_0 = dataCalibrated.r_zw_a(:,1);
v_zwa_a_0 = zeros(3,1); %dataCalibrated.v_zwa_a(:,1);
C_ba_0 = dataCalibrated.C_ba(:,:,1);
g_a = results.g_a;

traj_so3 = imuDeadReckoning(dataCalibrated, r_zw_a_0, v_zwa_a_0, C_ba_0, g_a, 'so3');
traj_rk4 = imuDeadReckoning(dataCalibrated, r_zw_a_0, v_zwa_a_0, C_ba_0, g_a, 'rk4');

phi_ba = DCM_TO_ROTVEC(traj_so3.C_ba);
phi_ba_mocap = DCM_TO_ROTVEC(dataCalibrated.C_ba);
r_zw_a_rk4 = traj_rk4.r_zw_a;
r_zw_a_so3 = traj_so3.r_zw_a;

% 3D Plot
figure
plot3(r_zw_a_so3(1,1:end), r_zw_a_so3(2,1:end), r_zw_a_so3(3,1:end),'linewidth',2)
hold on
plot3(dataCalibrated.r_zw_a(1,1:end),...
      dataCalibrated.r_zw_a(2,1:end),...
      dataCalibrated.r_zw_a(3,1:end),'linewidth',2);
plot3(r_zw_a_rk4(1,1:end), r_zw_a_rk4(2,1:end), r_zw_a_rk4(3,1:end),'LineWidth',2)
hold off
axis vis3d
axis equal
legend('Dead-Reckon Solution','Ground Truth','RK4 Dead-Reckoning')
title('Position')
%axis([-1 4 -2 2 0 3])
xlabel('x')
ylabel('y')
zlabel('z')
grid on

% Position Components
figure
subplot(3,1,1)
plot(dataCalibrated.t, r_zw_a_so3(1,:) - dataCalibrated.r_zw_a(1,:),'LineWidth',2)
grid on
ylabel('$\phi_1$','interpreter','latex')
title('Position Dead-reckoning Error')
axis([-inf inf -5 5])

subplot(3,1,2)
plot(dataCalibrated.t, r_zw_a_so3(2,:) - dataCalibrated.r_zw_a(2,:),'LineWidth',2)
grid on
ylabel('$\phi_2$','interpreter','latex')
axis([-inf inf -5 5])

subplot(3,1,3)
plot(dataCalibrated.t, r_zw_a_so3(3,:) - dataCalibrated.r_zw_a(3,:),'LineWidth',2)
grid on
ylabel('$\phi_3$','interpreter','latex')
axis([-inf inf -5 5])

% Attitude Components
figure
subplot(3,1,1)
plot(dataCalibrated.t, phi_ba(1,:))
hold on
plot(dataCalibrated.t, phi_ba_mocap(1,:))
hold off
grid on
ylabel('$\phi_1$','interpreter','latex')
legend('Dead-Reckoning','Ground Truth')
title('Attitude Dead-reckoning Performance')

subplot(3,1,2)
plot(dataCalibrated.t, phi_ba(2,:))
hold on
plot(dataCalibrated.t, phi_ba_mocap(2,:))
hold off
grid on
ylabel('$\phi_2$','interpreter','latex')

subplot(3,1,3)
plot(dataCalibrated.t, phi_ba(3,:))
hold on
plot(dataCalibrated.t, phi_ba_mocap(3,:))
hold off
grid on
ylabel('$\phi_3$','interpreter','latex')
