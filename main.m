% An example of how to use the Mocap tools.

clear; close all;

addpath('utils/')
addpath('calibrationData/')

%% Extract position of IMU relative to reference point
[r_imu2z_b, r_imu1z_b] = mocap_getPointInBodyFrame('IMU_Position_Calibration_Take_001.csv',...
                                          {'RigidBody002', 'RigidBody'},...
                                          {'Unlabeled1354', 'Unlabeled1355'});

%% Extract position of UWB tags relative to IMU
[r_Ltag2z_b, r_Rtag2z_b, r_Ltag1z_b, r_Rtag1z_b] = ...
         mocap_getPointInBodyFrame('Tag_Position_Calibration_Take001.csv',...
                          {'RigidBody002', 'RigidBody002', 'RigidBody', 'RigidBody'},...
                          {'Unlabeled1343', 'Unlabeled1342', 'Unlabeled1340', 'Unlabeled1341'});

% extract distance from IMU to UWB tags of the moving agent, in the body
% frame specified by the Mocap system
r_Ltag2imu2_b = r_Ltag2z_b - r_imu2z_b;
r_Rtag2imu2_b = r_Rtag2z_b - r_imu2z_b;

% extract distance from IMU to UWB tags of the static agent, in the body
% frame specified by the Mocap system
r_Ltag1imu1_b = r_Ltag1z_b - r_imu1z_b;
r_Rtag1imu1_b = r_Rtag1z_b - r_imu1z_b;

%% Extract Mocap data
dataMocap = mocap_csv2struct('Sensor_Frame_Calibration_Take_002.csv')

%% Align the reference point and the IMU
% TODO: 1) include in mocap_csv2struct?
for lv1=1:1:length(dataMocap.RigidBody002.t)
    dataMocap.RigidBody002.r_zw_a(:,lv1) = dataMocap.RigidBody002.r_zw_a(:,lv1) + ...
                                           dataMocap.RigidBody002.C_ba(:,:,lv1)'*r_imu2z_b;
end

%% Fit a b-spline to the Mocap data
splineMocap = mocap_fitSpline(dataMocap,[],true)

%% Extract the IMU data
dataIMU = IMU_csv2struct('2020_07_15_trial2_mmagent1_imu_sensorframe_calibration.csv')

%% Synchronize the Mocap and IMU data
dataSynced = syncTime(splineMocap.RigidBody002, dataIMU)

%% Obtain indices of gaps in mocap data, and identified stationary periods
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);

%% Align the frames of the Mocap and IMU data to find an initial DCM
tic
dataAligned = alignFrames(dataSynced)
toc

%% Refine the DCM between the Mocap assigned body frame and the IMU frame
options.frames = true;
options.bias = true;
options.scale = false;
options.skew = false;
options.grav = true;
[results, dataCalibrated] = calibrateImu(dataAligned, options)

%% Refine the DCM between the Mocap assigned body frame and the magnetometer frame
C_all = getAllPermutationMatrices(0);
dataCalibratedStatic = dataCalibrated;
min_cost = Inf;
for lv2 = 44:1:44
   close all
   
   dataCalibratedStatic.mag = C_all(:,:,lv2)*dataCalibrated.mag;
   [magResults, ~, cost] = magCalibrate(dataCalibratedStatic, options)
   
   if cost < min_cost
       min_cost = cost;
       align_final = C_all(:,:,lv2);
       magResults_final = magResults;
   end
end
