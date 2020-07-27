% An example of how to use the Mocap tools.

clear; close all;

addpath('utils/')
addpath('calibrationData/')

%% Extract Mocap data
dataMocap = mocap_csv2struct('Sensor_Frame_Calibration_Take_002.csv')

%% Fit a b-spline to the Mocap data
spline = mocap_fitSpline(dataMocap,[],true)

%% Extract the IMU data
dataIMU = IMU_csv2struct('2020_07_15_trial2_mmagent1_imu_sensorframe_calibration.csv')

%% Synchronize the Mocap and IMU data
dataSynced = syncTime(spline.RigidBody002, dataIMU)

%% Disregard IMU data within the time ranges where no ground truth was collected
dataSyncedCleaned = deleteGaps(dataSynced, dataMocap.RigidBody002.mocapGaps)

%% Align the frames of the Mocap and IMU data to find an initial DCM
[C_sm0, dataAligned] = alignFrames(dataSyncedCleaned)

%% Refine the DCM between the two assigned body frames
phi0 = DCM_TO_ROTVEC(C_sm0);
% [C_sm, biasAcc, biasGyr] = calibrateFrames(dataAligned, phi0);
x0 = [phi0;0;0;0;0;0;0];
[C_sm, biasAcc, biasGyr] = calibrateFrames(dataAligned, x0)
