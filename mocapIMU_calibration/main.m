% An example of how to use the Mocap tools.

addpath('utils/')
addpath('data/')

% Extract Mocap data
dataMocap = mocap_csv2struct('Sensor_Frame_Calibration_Take_002.csv')

% Fit a b-spline to the Mocap data
spline = mocap_fitSpline(dataMocap)

% Extract the IMU data
dataIMU = IMU_csv2struct('2020_07_15_trial2_mmagent1_imu_sensorframe_calibration.csv')

% Synchronize the Mocap and IMU data
dataSynced = syncTime(spline.RigidBody002, dataIMU)

% Find the DCM between the two assigned body frames
C_sm = calibrateFrames(dataSynced)