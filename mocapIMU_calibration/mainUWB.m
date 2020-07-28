% An example of how to extract UWB data and calibrate their biases.

clear; close all;

%% Extract Mocap data
dataMocap = mocap_csv2struct('Sensor_Frame_Calibration_Take_002.csv')

%% Extract the IMU data
[dataIMU, imuFirst_t] = IMU_csv2struct('2020_07_15_trial2_mmagent1_imu_sensorframe_calibration.csv')

%% Extract the UWB data
% To synchronize the IMU and the UWB files, ensure that "imuFirst_t" comes
% from the same device that timestamped the UWB measurements.
dataUWB = UWB_csv2struct('2020_07_15_trial3_mmagent1_range.csv', imuFirst_t)

%% Extract the UWB biases
