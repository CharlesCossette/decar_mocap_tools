%% Test - IMU calibration procedure with EuRoC MAV Dataset

load euroc_data_imu
load euroc_data_groundtruth

% Extract ground truth from dataset
dataImuRaw = table2struct(data_imu, 'ToScalar',true);
dataMocapRaw = table2struct(data_groundtruth, 'ToScalar', true);
dataMocap.RigidBody.t = (dataMocapRaw.timestamp(:)-dataMocapRaw.timestamp(1)).'*1e-9;
dataMocap.RigidBody.r_zw_a =  [dataMocapRaw.p_RS_R_xm.';
                               dataMocapRaw.p_RS_R_ym.';
                               dataMocapRaw.p_RS_R_zm.'];
dataMocap.RigidBody.q_ba =  [dataMocapRaw.q_RS_w.';
                             dataMocapRaw.q_RS_x.';
                             dataMocapRaw.q_RS_y.';
                             dataMocapRaw.q_RS_z.'];
% Smooth the quaternions 
dataMocap.RigidBody.C_ba = quat2dcm(dataMocap.RigidBody.q_ba.');
C_bprimeb = [0 0 -1; 0 1 0; 1 0 0];
%C_bprimeb = eye(3);
for lv1 = 1:size(dataMocap.RigidBody.C_ba,3)
    dataMocap.RigidBody.C_ba(:,:,lv1) = C_bprimeb*dataMocap.RigidBody.C_ba(:,:,lv1);
end
dataMocap.RigidBody.q_ba = dcmToQuat(dataMocap.RigidBody.C_ba);
dataMocap.RigidBody.type = 'Rigid Body';                       
dataMocap.RigidBody.gapIntervals = mocapGetGapIntervals(dataMocap.RigidBody);
dataMocap.RigidBody.staticIntervals = ...
    mocapGetStaticIntervals(dataMocap.RigidBody,2,0.0005);

dataIMU.t = (dataImuRaw.timestampns - dataImuRaw.timestampns(1)).'*1e-9;
dataIMU.gyro = C_bprimeb*[dataImuRaw.w_RS_S_xrads1.';
                dataImuRaw.w_RS_S_yrads1.';
                dataImuRaw.w_RS_S_zrads1.'];
dataIMU.accel = C_bprimeb*[dataImuRaw.a_RS_S_xms2.';
                 dataImuRaw.a_RS_S_yms2.';
                 dataImuRaw.a_RS_S_zms2.'];
%%           
splineMocap = mocap_fitSpline(dataMocap,[],true);

dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)
%%
options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
[results, dataCalibrated] = calibrateImu(dataAligned, options)