%% Test 1 - Simulated Data
clear;
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate imu/mocap data.
params.C_ms_accel = eye(3);
params.C_ms_gyro = eye(3);
params.bias_accel = [0;0;0];
params.bias_gyro = [0;0;0];
params.scale_accel = [1;1;1];
params.scale_gyro = [1;1;1];
params.skew_accel = [0;0;0];
params.skew_gyro = [0;0;0];
params.mocap_gravity = [0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 120;
params.imu_frequency = 250;
[dataMocap, dataIMU] = simulateTestData(params);


windowSize = 2;
stdDevThreshold = 0.002;
staticIntervals = mocapGetStaticIntervals(dataMocap.RigidBody,windowSize,stdDevThreshold);

t = dataMocap.RigidBody.t;
r_zw_a = dataMocap.RigidBody.r_zw_a;
staticIndices = getIndicesFromIntervals(t, staticIntervals);

figure(1)
plot(t,vecnorm(r_zw_a),'linewidth',2);
hold on
plot(t,4*staticIndices,'linewidth',2);
hold off
grid on
legend('Norm of Position (m)','Static Detector')
xlabel('Time (s)')

%% Test 2 - Real Data
% TODO: add asserts
dataMocap = mocap_csv2struct('2020_08_04_180_mocap_trial9.csv');
t = dataMocap.RigidBody.t;
r_zw_a = dataMocap.RigidBody.r_zw_a;
staticIndices = getIndicesFromIntervals(t, dataMocap.RigidBody.staticIntervals);

figure(1)
plot(t,vecnorm(r_zw_a),'linewidth',2);
hold on
plot(t,4*staticIndices,'linewidth',2);
hold off
grid on
legend('Norm of Position (m)','Static Detector')
xlabel('Time (s)')

%% Test 3 - Real Data
% TODO: add asserts
dataMocap = mocap_csv2struct('2020_08_04_180_mocap_trial6.csv');
t = dataMocap.RigidBody.t;
r_zw_a = dataMocap.RigidBody.r_zw_a;
staticIndices = getIndicesFromIntervals(t, dataMocap.RigidBody.staticIntervals);

figure(1)
plot(t,vecnorm(r_zw_a),'linewidth',2);
hold on
plot(t,4*staticIndices,'linewidth',2);
hold off
grid on
legend('Norm of Position (m)','Static Detector')
xlabel('Time (s)')