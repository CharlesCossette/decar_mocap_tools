% An example of how to use calibrate an IMU and magnetometer.

clear; close all;

addpath('utils/')
addpath('calibrationData/')

%% Extract position of IMU relative to reference point
[r_imu2z_b, r_imu1z_b] = mocapGetPointInBodyFrame('IMU_Position_Calibration_Take_001.csv',...
                                          {'RigidBody002', 'RigidBody'},...
                                          {'Unlabeled1354', 'Unlabeled1355'});

%% Extract Mocap data
dataMocap = mocapCsvToStruct('Sensor_Frame_Calibration_Take_002.csv')

%% Align the reference point and the IMU
% TODO: 1) include in mocap_csv2struct?
for lv1=1:1:length(dataMocap.RigidBody002.t)
    dataMocap.RigidBody002.r_zw_a(:,lv1) = dataMocap.RigidBody002.r_zw_a(:,lv1) + ...
                                           dataMocap.RigidBody002.C_ba(:,:,lv1)'*r_imu2z_b;
end

%% Fit a b-spline to the Mocap data
spline_mocap = mocapFitSpline(dataMocap,[],true)

%% Extract the IMU data
data_imu = sensorsCsvToStruct('2020_07_15_trial2_mmagent1_imu_sensorframe_calibration.csv')

%% Synchronize the Mocap and IMU data
data_synced = imuMocapSync(spline_mocap.RigidBody002, data_imu)

%% Align the frames of the Mocap and IMU data to find an initial DCM
tic
data_aligned = alignFrames(data_synced)
toc

%% Refine the DCM between the Mocap assigned body frame and the IMU frame
options.frames = true;
options.bias = true;
options.scale = false;
options.skew = false;
options.grav = true;
[results, dataCalibrated] = imuCalibrate(data_aligned, options)

%% Refine the DCM between the Mocap assigned body frame and the magnetometer frame
C_all = getAllPermutationMatrices(0);
dataCalibratedStatic = dataCalibrated;
min_cost = Inf;
for lv2 = 25:1:25
   close all
   
   dataCalibratedStatic.mag = C_all(:,:,lv2)*dataCalibrated.mag;
   [magResults, ~, cost] = magCalibrate(dataCalibratedStatic, options);
   
   if cost < min_cost
       min_cost = cost;
       align_final = C_all(:,:,lv2);
       magResults_final = magResults;
   end
end
