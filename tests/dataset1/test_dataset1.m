% An example of how to use the Mocap tools.

clear; close all;
%% Get IMU position
r_imuz_b = mocap_getPointInBodyFrame('2020_08_03_180_position_calibration.csv',...
                                     'RigidBody','Unlabeled2280')
%% Extract Mocap data
dataMocap = mocap_csv2struct('2020_08_03_180_mocap.csv')

%% Align the reference point and the IMU
% TODO: 1) include in mocap_csv2struct?
for lv1=1:1:length(dataMocap.RigidBody.t)
    dataMocap.RigidBody.r_zw_a(:,lv1) = dataMocap.RigidBody.r_zw_a(:,lv1) + ...
                                           dataMocap.RigidBody.C_ba(:,:,lv1).'*r_imuz_b;
end

%% Fit a b-spline to the Mocap data
spline = mocap_fitSpline(dataMocap,[],true)

%% Extract the IMU data
dataIMU = IMU_csv2struct('2020_07_28_21_47_34_sensehat.csv')

%% Synchronize the Mocap and IMU data
tic
dataSynced = syncTime(spline.RigidBody, dataIMU)
toc
%% Disregard IMU data within the time ranges where no ground truth was collected
dataSyncedCleaned = deleteGaps(dataSynced, dataMocap.RigidBody.mocapGaps)

%% Align the frames of the Mocap and IMU data to find an initial DCM
tic
dataAligned = alignFrames(dataSyncedCleaned)
toc
%% Refine the DCM between the two assigned body frames

[C_sm, biasAcc, biasGyr] = calibrateFrames(dataAligned)
