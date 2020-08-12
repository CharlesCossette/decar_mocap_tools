% An example of how to extract UWB data and calibrate their biases.

clear; close all;

%% Extract Mocap data
dataMocap = mocap_csv2struct('Sensor_Frame_Calibration_Take_001.csv')

%% Fit a b-spline to the Mocap data
splineMocap = mocap_fitSpline(dataMocap,[],true)

%% Extract the IMU data
[dataIMU, imuFirst_t] = IMU_csv2struct('2020_07_22_23_54_07_sensorframe_calibration_imu_rpi1.csv')

%% Synchronize the Mocap and IMU data
[~, offset] = syncTime(splineMocap.RigidBody, dataIMU);

% Adjust the spline such that it is also synchronized with the IMU/UWB
% data.
splineMocap.RigidBody002.breaks = splineMocap.RigidBody.breaks - offset;

%% Extract the UWB data
% To synchronize the IMU and the UWB files, ensure that "imuFirst_t" comes
% from the same device that timestamped the UWB measurements.
dataUWB = UWB_csv2struct('2020_07_22_23_54_06_sensorframe_calibration_range_rpi1.csv', imuFirst_t)

%% Extract position of UWB tags relative to the reference point of the Mocap System
% r_Ltag2z_b = 0x6a5d, r_Rtag2z_b = 0x6a26, r_Ltag1z_b = 0x6a25, r_Rtag1z_b = 0x6a7b

% TODO: NEED TO EXTRACT UNLABELED MARKERS FROM TAKE FILE IN LAB AND UPDATE THIS ACCORDINGLY.

% [r_Ltag2z_b, r_Rtag2z_b, r_Ltag1z_b, r_Rtag1z_b] = ...
%          mocap_getPointInBodyFrame('Tag_Position_Calibration_Take_001.csv',...
%                           {'RigidBody002', 'RigidBody002', 'RigidBody', 'RigidBody'},...
%                           {'Unlabeled1343', 'Unlabeled1342', 'Unlabeled1340', 'Unlabeled1341'});
                      
                      
%% Extract the UWB biases

% TODO: NEED TO EXTRACT UNLABELED MARKERS FROM TAKE FILE IN LAB AND UPDATE THIS ACCORDINGLY.

% dataUwbCorrected = extractUwbBias(splineMocap, dataUWB,...
%                                   [r_Ltag1z_b,   r_Rtag1z_b,  r_Ltag2z_b,     r_Rtag2z_b],...
%                                   {'RigidBody', 'RigidBody', 'RigidBody002', 'RigidBody002'},...
%                                   {'0x6a25',    '0x6a7b',    '0x6a5d',       '0x6a26'});