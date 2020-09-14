%% Test 1 - No noise, no corruption at all

phi = [0.1;0.2;0.3]*0;
sim_params.C_ms_accel = ROTVEC_TO_DCM(phi);
sim_params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
sim_params.bias_accel = [-0.1;0.2;0.3]*0;
sim_params.bias_gyro = -[-0.1;0.2;0.3]*0;
sim_params.scale_accel = [1;1;1];
sim_params.scale_gyro = [1;1;1];
sim_params.skew_accel = [0.1;0;0]*0;
sim_params.skew_gyro = [0;-0.2;0]*0;
sim_params.mocap_gravity = [0;0;-9.80665];
sim_params.std_dev_accel = 0;
sim_params.std_dev_gyro = 0;
sim_params.mocap_frequency = 100;
sim_params.imu_frequency = 100;
sim_params.sim_duration = 10;
[dataMocap, dataIMU] = simulateTestData(sim_params);

C_ma = sim_params.C_ms_accel;
C_mg = sim_params.C_ms_gyro;
bias_a = sim_params.bias_accel;
bias_g = sim_params.bias_gyro;
scale_a = sim_params.scale_accel;
scale_g = sim_params.scale_gyro;
skew_a = sim_params.skew_accel;
skew_g = sim_params.skew_gyro;
C_ae = eye(3);

dataSynced = dataIMU;
dataSynced.r_zw_a = dataMocap.RigidBody.r_zw_a;
dataSynced.v_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.a_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.C_ba = dataMocap.RigidBody.C_ba;
dataSynced.q_ba = dataMocap.RigidBody.q_ba;
dataSynced.gapIndices = false(size(dataSynced.t));
dataSynced.staticIndices = false(size(dataSynced.t));
dataSynced.gyro_mocap = dataSynced.gyro;
dataSynced.accel_mocap = dataSynced.accel;
error_params.interval_size = 30000;
error_params.batch_size = 30000;
error_params.min_index = 1;
error_params.max_index = length(dataIMU.t);

e = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                          scale_a, scale_g, skew_a, skew_g,...
                          C_ae, dataSynced, error_params);
max(abs(e))                      
assert(max(abs(e)) < 1e-13);

%% Test 2 - DCM Corruption

phi = [0.1;0.2;0.3];
sim_params.C_ms_accel = ROTVEC_TO_DCM(phi);
sim_params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
sim_params.bias_accel = [-0.1;0.2;0.3]*0;
sim_params.bias_gyro = -[-0.1;0.2;0.3]*0;
sim_params.scale_accel = [1;1;1];
sim_params.scale_gyro = [1;1;1];
sim_params.skew_accel = [0.1;0;0]*0;
sim_params.skew_gyro = [0;-0.2;0]*0;
sim_params.mocap_gravity = [0;0;-9.80665];
sim_params.std_dev_accel = 0;
sim_params.std_dev_gyro = 0;
sim_params.mocap_frequency = 100;
sim_params.imu_frequency = 100;
sim_params.sim_duration = 10;
[dataMocap, dataIMU] = simulateTestData(sim_params);

C_ma = sim_params.C_ms_accel;
C_mg = sim_params.C_ms_gyro;
bias_a = sim_params.bias_accel;
bias_g = sim_params.bias_gyro;
scale_a = sim_params.scale_accel;
scale_g = sim_params.scale_gyro;
skew_a = sim_params.skew_accel;
skew_g = sim_params.skew_gyro;
C_ae = eye(3);

dataSynced = dataIMU;
dataSynced.r_zw_a = dataMocap.RigidBody.r_zw_a;
dataSynced.v_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.a_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.C_ba = dataMocap.RigidBody.C_ba;
dataSynced.q_ba = dataMocap.RigidBody.q_ba;
dataSynced.gapIndices = false(size(dataSynced.t));
dataSynced.staticIndices = false(size(dataSynced.t));
dataSynced.gyro_mocap = dataSynced.gyro;
dataSynced.accel_mocap = dataSynced.accel;
error_params.interval_size = 30000;
error_params.batch_size = 30000;
error_params.min_index = 1;
error_params.max_index = length(dataIMU.t);

e = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                          scale_a, scale_g, skew_a, skew_g,...
                          C_ae, dataSynced, error_params);
max(abs(e))                    
assert(max(abs(e)) < 1e-12);

%% Test 3 - DCM, Bias Corruption
phi = [0.1;0.2;0.3];
sim_params.C_ms_accel = ROTVEC_TO_DCM(phi);
sim_params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
sim_params.bias_accel = [-0.1;0.2;0.3];
sim_params.bias_gyro = -[-0.1;0.2;0.3];
sim_params.scale_accel = [1;1;1];
sim_params.scale_gyro = [1;1;1];
sim_params.skew_accel = [0.1;0;0]*0;
sim_params.skew_gyro = [0;-0.2;0]*0;
sim_params.mocap_gravity = [0;0;-9.80665];
sim_params.std_dev_accel = 0;
sim_params.std_dev_gyro = 0;
sim_params.mocap_frequency = 100;
sim_params.imu_frequency = 100;
sim_params.sim_duration = 10;
[dataMocap, dataIMU] = simulateTestData(sim_params);

C_ma = sim_params.C_ms_accel;
C_mg = sim_params.C_ms_gyro;
bias_a = sim_params.bias_accel;
bias_g = sim_params.bias_gyro;
scale_a = sim_params.scale_accel;
scale_g = sim_params.scale_gyro;
skew_a = sim_params.skew_accel;
skew_g = sim_params.skew_gyro;
C_ae = eye(3);

dataSynced = dataIMU;
dataSynced.r_zw_a = dataMocap.RigidBody.r_zw_a;
dataSynced.v_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.a_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.C_ba = dataMocap.RigidBody.C_ba;
dataSynced.q_ba = dataMocap.RigidBody.q_ba;
dataSynced.gapIndices = false(size(dataSynced.t));
dataSynced.staticIndices = false(size(dataSynced.t));
dataSynced.gyro_mocap = dataSynced.gyro;
dataSynced.accel_mocap = dataSynced.accel;
error_params.interval_size = 30000;
error_params.batch_size = 30000;
error_params.min_index = 1;
error_params.max_index = length(dataIMU.t);

e = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                          scale_a, scale_g, skew_a, skew_g,...
                          C_ae, dataSynced, error_params);
max(abs(e))                      
assert(max(abs(e)) < 1e-12);

%% Test 4 - DCM, Bias, Scale Corruption
phi = [0.1;0.2;0.3];
sim_params.C_ms_accel = ROTVEC_TO_DCM(phi);
sim_params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
sim_params.bias_accel = [-0.1;0.2;0.3];
sim_params.bias_gyro = -[-0.1;0.2;0.3];
sim_params.scale_accel = [3;1.5;0.9];
sim_params.scale_gyro = [0.8;1;1.2];
sim_params.skew_accel = [0.1;0;0]*0;
sim_params.skew_gyro = [0;-0.2;0]*0;
sim_params.mocap_gravity = [0;0;-9.80665];
sim_params.std_dev_accel = 0;
sim_params.std_dev_gyro = 0;
sim_params.mocap_frequency = 100;
sim_params.imu_frequency = 100;
sim_params.sim_duration = 10;
[dataMocap, dataIMU] = simulateTestData(sim_params);

C_ma = sim_params.C_ms_accel;
C_mg = sim_params.C_ms_gyro;
bias_a = sim_params.bias_accel;
bias_g = sim_params.bias_gyro;
scale_a = sim_params.scale_accel;
scale_g = sim_params.scale_gyro;
skew_a = sim_params.skew_accel;
skew_g = sim_params.skew_gyro;
C_ae = eye(3);

dataSynced = dataIMU;
dataSynced.r_zw_a = dataMocap.RigidBody.r_zw_a;
dataSynced.v_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.a_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.C_ba = dataMocap.RigidBody.C_ba;
dataSynced.q_ba = dataMocap.RigidBody.q_ba;
dataSynced.gapIndices = false(size(dataSynced.t));
dataSynced.staticIndices = false(size(dataSynced.t));
dataSynced.gyro_mocap = dataSynced.gyro;
dataSynced.accel_mocap = dataSynced.accel;
error_params.interval_size = 30000;
error_params.batch_size = 30000;
error_params.min_index = 1;
error_params.max_index = length(dataIMU.t);

e = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                          scale_a, scale_g, skew_a, skew_g,...
                          C_ae, dataSynced, error_params);
max(abs(e))                      
assert(max(abs(e)) < 1e-12);

%% Test 5 - DCM, Bias, Scale, Skew Corruption
phi = [0.1;0.2;0.3];
sim_params.C_ms_accel = ROTVEC_TO_DCM(phi);
sim_params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
sim_params.bias_accel = [-0.1;0.2;0.3];
sim_params.bias_gyro = -[-0.1;0.2;0.3];
sim_params.scale_accel = [3;1.5;0.9];
sim_params.scale_gyro = [0.8;1;1.2];
sim_params.skew_accel = [0.1;0;1.1];
sim_params.skew_gyro = [1.2;-0.2;-5];
sim_params.mocap_gravity = [0;0;-9.80665];
sim_params.std_dev_accel = 0;
sim_params.std_dev_gyro = 0;
sim_params.mocap_frequency = 100;
sim_params.imu_frequency = 100;
sim_params.sim_duration = 10;
[dataMocap, dataIMU] = simulateTestData(sim_params);

C_ma = sim_params.C_ms_accel;
C_mg = sim_params.C_ms_gyro;
bias_a = sim_params.bias_accel;
bias_g = sim_params.bias_gyro;
scale_a = sim_params.scale_accel;
scale_g = sim_params.scale_gyro;
skew_a = sim_params.skew_accel;
skew_g = sim_params.skew_gyro;
C_ae = eye(3);

dataSynced = dataIMU;
dataSynced.r_zw_a = dataMocap.RigidBody.r_zw_a;
dataSynced.v_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.a_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.C_ba = dataMocap.RigidBody.C_ba;
dataSynced.q_ba = dataMocap.RigidBody.q_ba;
dataSynced.gapIndices = false(size(dataSynced.t));
dataSynced.staticIndices = false(size(dataSynced.t));
dataSynced.gyro_mocap = dataSynced.gyro;
dataSynced.accel_mocap = dataSynced.accel;
error_params.interval_size = 30000;
error_params.batch_size = 30000;
error_params.min_index = 1;
error_params.max_index = length(dataIMU.t);

e = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                          scale_a, scale_g, skew_a, skew_g,...
                          C_ae, dataSynced, error_params);
max(abs(e))                      
assert(max(abs(e)) < 1e-12);

%% Test 5 - DCM, Bias, Scale, Skew, Gravity Corruption
phi = [0.1;0.2;0.3];
sim_params.C_ms_accel = ROTVEC_TO_DCM(phi);
sim_params.C_ms_gyro = ROTVEC_TO_DCM(-phi);
sim_params.bias_accel = [-0.1;0.2;0.3];
sim_params.bias_gyro = -[-0.1;0.2;0.3];
sim_params.scale_accel = [3;1.5;0.9];
sim_params.scale_gyro = [0.8;1;1.2];
sim_params.skew_accel = [0.1;0;1.1];
sim_params.skew_gyro = [1.2;-0.2;-5];
sim_params.mocap_gravity =  ROTVEC_TO_DCM(phi)*[0;0;-9.80665];
sim_params.std_dev_accel = 0;
sim_params.std_dev_gyro = 0;
sim_params.mocap_frequency = 100;
sim_params.imu_frequency = 100;
sim_params.sim_duration = 10;
[dataMocap, dataIMU] = simulateTestData(sim_params);

C_ma = sim_params.C_ms_accel;
C_mg = sim_params.C_ms_gyro;
bias_a = sim_params.bias_accel;
bias_g = sim_params.bias_gyro;
scale_a = sim_params.scale_accel;
scale_g = sim_params.scale_gyro;
skew_a = sim_params.skew_accel;
skew_g = sim_params.skew_gyro;
C_ae = ROTVEC_TO_DCM(phi);

dataSynced = dataIMU;
dataSynced.r_zw_a = dataMocap.RigidBody.r_zw_a;
dataSynced.v_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.a_zwa_a = zeros(size(dataSynced.r_zw_a));
dataSynced.C_ba = dataMocap.RigidBody.C_ba;
dataSynced.q_ba = dataMocap.RigidBody.q_ba;
dataSynced.gapIndices = false(size(dataSynced.t));
dataSynced.staticIndices = false(size(dataSynced.t));
dataSynced.gyro_mocap = dataSynced.gyro;
dataSynced.accel_mocap = dataSynced.accel;
error_params.interval_size = 30000;
error_params.batch_size = 30000;
error_params.min_index = 1;
error_params.max_index = length(dataIMU.t);

e = imuDeadReckoningError(C_ma, C_mg, bias_a, bias_g,...
                          scale_a, scale_g, skew_a, skew_g,...
                          C_ae, dataSynced, error_params);
max(abs(e))                      
assert(max(abs(e)) < 1e-12);