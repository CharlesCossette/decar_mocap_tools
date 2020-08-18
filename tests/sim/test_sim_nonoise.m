%% Test 1 - No noise, no corruption at all
clear;
close all

phi = [0.1;0.2;0.3]*0;
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3]*0;
params.bias_gyro = -[-0.1;0.2;0.3]*0;
params.scale_accel = [1;1;1];
params.scale_gyro = [1;1;1];
params.skew_accel = [0.1;0;0]*0;
params.skew_gyro = [0;-0.2;0]*0;
params.mocap_gravity = [0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 100;
params.imu_frequency = 100;
params.sim_duration = 10;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 2 - DCM Corruption
clear; close all;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3]*0;
params.bias_gyro = -[-0.1;0.2;0.3]*0;
params.scale_accel = [1;1;1];
params.scale_gyro = [1;1;1];
params.skew_accel = [0.1;0;0]*0;
params.skew_gyro = [0;-0.2;0]*0;
params.mocap_gravity = [0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 100;
params.imu_frequency = 100;
params.sim_duration = 100;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = false;
options.scale = false;
options.skew = false;
options.grav = false;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 3 - DCM, Bias Corruption
clear; close all;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1;1;1];
params.scale_gyro = [1;1;1];
params.skew_accel = [0.1;0;0]*0;
params.skew_gyro = [0;-0.2;0]*0;
params.mocap_gravity = [0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 100;
params.imu_frequency = 100;
params.sim_duration = 10;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = true;
options.scale = false;
options.skew = false;
options.grav = false;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 4 - DCM, Bias, Scale Corruption
clear; close all;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1.1;1;0.9];
params.scale_gyro = [1;0.8;1.4];
params.skew_accel = [0.1;0;0]*0;
params.skew_gyro = [0;-0.2;0]*0;
params.mocap_gravity = [0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 100;
params.imu_frequency = 100;
params.sim_duration = 10;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = true;
options.scale = true;
options.skew = false;
options.grav = false;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 5 - DCM, Bias, Scale, Skew Corruption
clear; close all;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1.1;1;0.9];
params.scale_gyro = [1;0.8;1.4];
params.skew_accel = [0.1;0;0.1];
params.skew_gyro = [0;-0.2;0];
params.mocap_gravity = [0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 100;
params.imu_frequency = 100;
params.sim_duration = 10;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
options.grav = false;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 6 - DCM, Bias, Scale, Skew, Gravity Corruption
clear; close all;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1.1;1;0.9];
params.scale_gyro = [1;0.8;1.4];
params.skew_accel = [0.1;0;0.1];
params.skew_gyro = [0;-0.2;0];
params.mocap_gravity = ROTVEC_TO_DCM(phi)*[0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 100;
params.imu_frequency = 100;
params.sim_duration = 10;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 7 - Full Corruption, 40s duration
clear;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1.1;1;0.9];
params.scale_gyro = [1;0.8;1.4];
params.skew_accel = [0.1;0;0.1];
params.skew_gyro = [0;-0.2;0];
params.mocap_gravity = ROTVEC_TO_DCM(phi)*[0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 100;
params.imu_frequency = 100;
params.sim_duration = 40;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 7 - Full Corruption, Mini-batches
clear;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1.1;1;0.9];
params.scale_gyro = [1;0.8;1.4];
params.skew_accel = [0.1;0;0.1];
params.skew_gyro = [0;-0.2;0];
params.mocap_gravity = ROTVEC_TO_DCM(phi)*[0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 100;
params.imu_frequency = 100;
params.sim_duration = 100;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
options.interval_size = 500;
options.batch_size = 500;

[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 8 - Noise, Full Corruption, Mini Batches
clear;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1.1;1;0.9];
params.scale_gyro = [1;0.8;1.4];
params.skew_accel = [0.1;0;0.1];
params.skew_gyro = [0;-0.2;0];
params.mocap_gravity = ROTVEC_TO_DCM(phi)*[0;0;-9.80665];
params.std_dev_accel = 0.01;
params.std_dev_gyro = 0.01;
params.mocap_frequency = 100;
params.imu_frequency = 100;
params.sim_duration = 120;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)


options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
options.interval_size = 500;
options.batch_size = 500;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 9 - Noise, Full Corruption, Mini Batches, Higher Frequency
clear;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1.1;1;0.9];
params.scale_gyro = [1;0.8;1.4];
params.skew_accel = [0.1;0;0.1];
params.skew_gyro = [0;-0.2;0];
params.mocap_gravity = ROTVEC_TO_DCM(phi)*[0;0;-9.80665];
params.std_dev_accel = 0.01;
params.std_dev_gyro = 0.01;
params.mocap_frequency = 250;
params.imu_frequency = 250;
params.sim_duration = 120;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)


options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
options.interval_size = 2000;
options.batch_size = 500;
options.max_total_states = 50000;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 10 - Full Corruption, Asynchronous
clear; 
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1.1;1;0.9];
params.scale_gyro = [1;0.8;1.4];
params.skew_accel = [0.1;0;0.1];
params.skew_gyro = [0;-0.2;0];
params.mocap_gravity = ROTVEC_TO_DCM(phi)*[0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 120;
params.imu_frequency = 250;
params.sim_duration = 40;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)

options.frames = true;
options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity

%% Test 11 - Noise, Full Corruption, Mini Batches, Asynchronous
clear;
phi = [0.1;0.2;0.3];
params.C_ms_accel = ROTVEC_TO_DCM(phi);
params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
params.bias_accel = [-0.1;0.2;0.3];
params.bias_gyro = -[-0.1;0.2;0.3]*0.1;
params.scale_accel = [1.1;1;0.9];
params.scale_gyro = [1;0.8;1.4];
params.skew_accel = [0.1;0;0.1];
params.skew_gyro = [0;-0.2;0];
params.mocap_gravity = ROTVEC_TO_DCM(phi)*[0;0;-9.80665];
params.std_dev_accel = 0.01;
params.std_dev_gyro = 0.01;
params.mocap_frequency = 120;
params.imu_frequency = 250;
params.sim_duration = 120;

[dataMocap, dataIMU] = simulateTestData(params)

splineMocap = mocap_fitSpline(dataMocap,[],true)
dataSynced = syncTime(splineMocap.RigidBody, dataIMU,12,true)
dataSynced.v_zwa_a(:,1) = zeros(3,1); % WE KNOW THIS FROM SIM.
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
dataAligned = alignFrames(dataSynced)


options.bias = true;
options.scale = true;
options.skew = true;
options.grav = true;
options.interval_size = 2000;
options.batch_size = 500;
options.max_total_states = 50000;
[results, dataCalibrated] = calibrateImu(dataSynced,options)

results.C_ms_accel - params.C_ms_accel
results.C_ms_gyro - params.C_ms_gyro
results.bias_accel - params.bias_accel
results.bias_gyro - params.bias_gyro
results.scale_accel - params.scale_accel
results.scale_gyro - params.scale_gyro
results.skew_accel - params.skew_accel
results.skew_gyro - params.skew_gyro
results.g_a - params.mocap_gravity


% must_simulate = true;
% if exist('./tests/sim/test1_sim_data.mat','file')
%     load('./tests/sim/test1_sim_data')
%     must_simulate = false;
%     if ~exist('params_saved','var')
%         must_simulate = true;
%     else
%         if ~isequal(params, params_saved)
%             must_simulate = true;
%         end
%     end
% end
% if must_simulate
%     [dataMocap, dataIMU] = simulateTestData(params)
%     params_saved = params;
%     save('./tests/sim/test1_sim_data','params_saved','dataMocap','dataIMU')
% end
% 
