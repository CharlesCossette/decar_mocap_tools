clear; close all;

TAKE_FILENAME = '2020-08-31_3D_Testing_1.csv';
IMU_FILENAME = '2020_07_30_22_53_45_3D_testing_1';

%% Extract Mocap Data
data_mocap = mocapCsvToStruct(TAKE_FILENAME);

%% Extract IMU Data
data_imu = sensorsCsvToStruct(IMU_FILENAME);

%% Fit a spline to the mocap data
spline_mocap = mocapFitSpline(data_mocap);

%% Synchronize the IMU and Mocap
data_imu_synced = imuMocapSync(spline_mocap.Broomstick, data_imu.data1);

%% Do a coarse frame alignment step
% TODO: this step will be moved to inside imuCalibrate soon.
data_imu_aligned = alignFrames(data_imu_synced, true);

%% Calibrate the IMU
[results, data_imu_calibrated] = imuCalibrate(data_imu_aligned);