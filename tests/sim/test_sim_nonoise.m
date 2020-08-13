%% Test 1 - No noise, no corruption at all, 120Hz mocap, 250Hz Imu
clear;
close all

phi = 0*[0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3];
params.scale_accel = [1;1;1];
params.scale_gyro = [1;1;1];
params.skew_accel = [0.1;0;0]*0;
params.skew_gyro = [0;-0.2;0]*0;
params.mocap_gravity = [0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 120;
params.imu_frequency = 236;

must_simulate = false;
if exist('./tests/sim/test1_sim_data.mat','file')
    load('./tests/sim/test1_sim_data')
    if ~exist('params_saved','var')
        must_simulate = true;
    else
        if ~isequal(params, params_saved)
            must_simulate = true;
        end
    end
end
if must_simulate
    [dataMocap, dataIMU] = simulateTestData(params)
    params_saved = params;
    save('./tests/sim/test1_sim_data','params_saved','dataMocap','dataIMU')
end
dataIMU.t = dataIMU.t + 1.123;

splineMocap = mocap_fitSpline(dataMocap,5,true)
dataSynced = syncTime(splineMocap.RigidBody,dataIMU)
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)
%%

options.frames = true;
options.bias = true;
options.scale = true;
options.skew = false;
options.grav = true;
[results, dataCalibrated] = calibrateImu(dataSynced,options)
results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.g_a - params.mocap_gravity

%% Test 1 - No noise, bias, 120Hz mocap, 236Hz Imu
clear;
phi = [0.1;0.1;-0.2];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3];
params.scale_accel = [1.2;1.2;1.1];
params.scale_gyro = [1.05;0.95;1];
params.skew_accel = [0;0;0];
params.skew_gyro = [0;0;0];
g_a = [0;0;-1];
params.mocap_gravity = 9.80665*(g_a./norm(g_a));
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 120;
params.imu_frequency = 236;

must_simulate = false;
if exist('./tests/sim/test2_sim_data.mat','file')
    load('./tests/sim/test2_sim_data')
    if ~exist('params_saved','var')
        must_simulate = true;
    else
        if ~isequal(params, params_saved)
            must_simulate = true;
        end
    end
end
if must_simulate
    [dataMocap, dataIMU] = simulateTestData(params)
    params_saved = params;
    save('./tests/sim/test2_sim_data','params_saved','dataMocap','dataIMU')
end

splineMocap = mocap_fitSpline(dataMocap,5,true)
dataSynced = syncTime(splineMocap.RigidBody,dataIMU)
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
[results, dataCalibrated] = calibrateFrames(dataSynced,options)