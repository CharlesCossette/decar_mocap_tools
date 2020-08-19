function [data_mocap, data_imu] = simulateTestData(params)
%% IMU Corruption Parameters

C_ma = params.C_ms_accel;
C_mg = params.C_ms_gyro;
bias_a = params.bias_accel;
bias_g = params.bias_gyro;
scale_a = params.scale_accel;
scale_g = params.scale_gyro;
skew_a = params.skew_accel;
skew_g = params.skew_gyro;
g_a = params.mocap_gravity;
sigma_accel = params.std_dev_accel;
sigma_gyro = params.std_dev_gyro;
mocap_freq = params.mocap_frequency;
imu_freq = params.imu_frequency;

%% Construct sample acceleration trajectory
% Simulate the TRUE bodyframe accelerations/angular velocities.
rng(1)

N = 20;
accel_waypoints = randn(3,N); % a_zwa_b
omega_waypoints = randn(3,N)*0.3; % om_ba_b
accel_waypoints = [[0;0;0],accel_waypoints,[0;0;0]];
omega_waypoints = [[0;0;0],omega_waypoints,[0;0;0]];
times = linspace(0,params.sim_duration,N + 2);
pp_accel = spline(times,accel_waypoints);
pp_omega = spline(times,omega_waypoints);

%% Generate IMU Data

% Initial conditions
C_ba_0 = eye(3);

% Dead-reckon the attitude forward in time so we know how to add the
% gravity vector.
t_imu = linspace(0, params.sim_duration, params.sim_duration*imu_freq);
gyro_ideal = zeros(3,length(t_imu));
accel_ideal = zeros(3,length(t_imu));
C_ba = zeros(3,3,length(t_imu));
C_ba(:,:,1) = C_ba_0;
for lv1 = 1:length(t_imu) -1
    dt = t_imu(lv1 + 1) - t_imu(lv1);
    om_ba_b = ppval(pp_omega, t_imu(lv1));
    a_zwa_b = ppval(pp_accel, t_imu(lv1));
    accel_ideal(:,lv1) = a_zwa_b - C_ba(:,:,lv1)*g_a;
    gyro_ideal(:,lv1) = om_ba_b;    
    C_ba(:,:,lv1 + 1) = expmTaylor(-CrossOperator(om_ba_b*dt))*C_ba(:,:,lv1);
end
accel_ideal(:,end) = ppval(pp_accel, t_imu(end)) - C_ba(:,:,end)*g_a;
gyro_ideal(:,end) = ppval(pp_omega, t_imu(end));

% Corrupted Accelerometer measurements
T_skew = eye(3);
T_skew(1,2) = -skew_a(3);
T_skew(1,3) = skew_a(2);
T_skew(2,3) = -skew_a(1);
accel_corrupt = inv(diag(scale_a))*inv(T_skew)*C_ma.'*accel_ideal - bias_a ...
                + randn(size(accel_ideal))*sigma_accel;

% Corrupted Gyroscope measurements
T_skew = eye(3);
T_skew(1,2) = -skew_g(3);
T_skew(1,3) = skew_g(2);
T_skew(2,3) = -skew_g(1);
gyro_corrupt = inv(diag(scale_g))*inv(T_skew)*C_mg.'*gyro_ideal - bias_g ...
               + randn(size(gyro_ideal))*sigma_gyro;

data_imu.t = t_imu;
data_imu.accel= accel_corrupt;
data_imu.gyro = gyro_corrupt;

data_imu_ideal.t = t_imu;
data_imu_ideal.accel = accel_ideal;
data_imu_ideal.gyro = gyro_ideal;

%% Generate Mocap Data 

% Initial conditions
r_zw_a_0 = [0;0;0];
v_zwa_a_0 = [0;0;0];
C_ba_0 = eye(3);
%x_0 = [r_zw_a_0; v_zwa_a_0; C_ba_0(:)];

% Time Span
t_mocap = linspace(0, params.sim_duration, params.sim_duration*mocap_freq);

traj = imuDeadReckoning(data_imu_ideal, r_zw_a_0, v_zwa_a_0, C_ba_0, g_a,'so3',t_mocap);

% Plot the integrated trajectory as a visual check
figure(1)
r_zw_a = traj.r_zw_a;
plot3(r_zw_a(1,:), r_zw_a(2,:), r_zw_a(3,:),'Linewidth',2);
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
axis vis3d
grid on
title('True Trajectory')

% Store the Mocap data
data_mocap.RigidBody.r_zw_a = traj.r_zw_a;
data_mocap.RigidBody.t = t_mocap;
data_mocap.RigidBody.type = 'Rigid Body';
data_mocap.RigidBody.C_ba = traj.C_ba;
data_mocap.RigidBody.q_ba = dcmToQuat(traj.C_ba);
data_mocap.RigidBody.gapIntervals = [];
data_mocap.RigidBody.staticIntervals = mocapGetStaticIntervals(data_mocap.RigidBody,...
                                                              2,...
                                                              0.002);
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_dot, data] = kinematics(t,x,g_a, pp_accel, pp_omega)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Simulation of the truth
    r_zw_a = x(1:3);
    v_zwa_a = x(4:6);
    C_ba = reshape(x(7:15),3,3);
    
    a_zwa_b = ppval(pp_accel,t);
    a_zwa_a = C_ba.'*a_zw_a;
    om_ba_b = ppval(pp_omega,t);
    
    r_zw_a_dot = v_zwa_a;
    v_zw_a_dot = a_zwa_a;
    C_ba_dot = -CrossOperator(om_ba_b)*C_ba;
    x_dot = [r_zw_a_dot;v_zw_a_dot;C_ba_dot(:)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    data.r_zw_a = r_zw_a;
    data.v_zw_a = v_zwa_a;
    data.a_zw_a = a_zwa_a;
    data.u_acc_b = C_ba*(a_zwa_a - g_a);
    data.u_gyr_b = om_ba_b;
    data.C_ba = C_ba;
end
% %Integrate
% options = odeset('AbsTol',1e-16,'RelTol',1e-13);
% [t,x] = ode4(@(t,x)kinematics(t,x,g_a,pp), tspan, x_0, options);
% clear data
% for lv1 = 1:size(x,1)
%     [~,data(lv1)] = kinematics(t(lv1), x(lv1,:).', g_a, pp);
% end
% 
% % Extract ideal accel/gyro measurements.
% dataIMU.t = t;
% accel_ideal = [data(:).u_acc_b];
% omega_ideal = [data(:).u_gyr_b];
