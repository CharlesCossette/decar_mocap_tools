function [dataMocap, dataIMU] = simulateTestData(params)
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
rng(1)
% Generate random waypoints with a spike at the beginning

N = 20;
waypoints = randn(7,N)*3;
waypoints(4:7,:) = waypoints(4:7,:)./vecnorm(waypoints(4:7,:));
times = linspace(50,120,N);

% Need special set of waypoints to construct the spike at the beginning.
spike_waypoints = [repmat([0;0;0],1,10),...
                   interp1([30,31],[[0;0;0],[0.5;0.5;0]].',linspace(30,31,10)).',...
                   repmat([0.5;0.5;0],1,10)];
spike_waypoints = [spike_waypoints;repmat([1;0;0;0],1,size(spike_waypoints,2))];
spike_times = [linspace(0,29.9,10),linspace(30,30.5,10),linspace(30.6,40,10)];

% Combine to full list of waypoints
waypoints = [spike_waypoints,waypoints];
times = [spike_times,times];

pp = spline(times, waypoints);
%pp = fn2fm(spaps(times,waypoints,0.000001,[],2),'pp');

% Construct spline, visually inspect for an acceptable trajectory.
t = linspace(0,120,1000);
trajectory = ppval(pp,t);
trajectoryDerv = splineDerv(pp,t,1);
trajectoryDerv2 = splineDerv(pp,t,2);
figure
plot3(trajectory(1,:),trajectory(2,:), trajectory(3,:),'linewidth',2)
hold on
scatter3(waypoints(1,:),waypoints(2,:),waypoints(3,:))
hold off
axis vis3d
axis equal
grid on

figure
plot(t, vecnorm(trajectoryDerv2(1:3,:)))
grid on
xlabel('Time (s)')
ylabel('Norm of acceleration')

%% Generate Mocap Data 
% The above spline produces an acceleration/angular velocity profile that
% we will now integrate to create a "true trajectory" that is more
% realistic. We leverage ODE45's accurate integration process for this.
% The output of this is used to create the simulated mocap data.

% Initial conditions
r_zw_a_0 = waypoints(1:3,1);
v_zw_a_0 = trajectoryDerv(1:3,1);
C_ba_0 = quat2dcm(waypoints(4:7,1).');
x_0 = [r_zw_a_0;v_zw_a_0; C_ba_0(:)];

% Time Span
tspan = linspace(0,120,120*mocap_freq);

% Integrate
options = odeset('AbsTol',1e-16,'RelTol',1e-13);
[t,x] = ode45(@(t,x)kinematics(t,x,g_a,pp), tspan, x_0, options);
clear data
for lv1 = 1:size(x,1)
    [~,data(lv1)] = kinematics(t(lv1),x(lv1,:).',g_a,pp);
end

% Plot the integrated trajectory as a visual check
figure(1)
r_zw_a = [data(:).r_zw_a];
v_zw_a = [data(:).v_zw_a];
a_zw_a = [data(:).a_zw_a];
C_ba = reshape([data(:).C_ba],3,3,[]);
plot3(x(:,1), x(:,2), x(:,3),'Linewidth',2);
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
axis vis3d
grid on
title('True Trajectory')

% Store the M
dataMocap.RigidBody.r_zw_a = r_zw_a;
dataMocap.RigidBody.t = t;
dataMocap.RigidBody.type = 'Rigid Body';
dataMocap.RigidBody.C_ba = C_ba;
dataMocap.RigidBody.q_ba = smoothdcm2quat(C_ba).';
dataMocap.RigidBody.gapIntervals = [];
dataMocap.RigidBody.staticIntervals = [];

%% Generate IMU Data
% We again use ODE45 to simulate a "true" trajectory identically to the
% mocap data, except we now extract the acceleration and corrupt it with
% the above parameters.

% Initial conditions
r_zw_a_0 = waypoints(1:3,1);
v_zw_a_0 = trajectoryDerv(1:3,1);
C_ba_0 = quat2dcm(waypoints(4:7,1).');
x_0 = [r_zw_a_0;v_zw_a_0; C_ba_0(:)];

% For the IMU truth is again sampled but at higher frequency. 
tspan = linspace(0,120,120*imu_freq);

%Integrate
options = odeset('AbsTol',1e-16,'RelTol',1e-13);
[t,x] = ode45(@(t,x)kinematics(t,x,g_a,pp), tspan, x_0, options);
clear data
for lv1 = 1:size(x,1)
    [~,data(lv1)] = kinematics(t(lv1), x(lv1,:).', g_a, pp);
end

% Extract ideal accel/gyro measurements.
dataIMU.t = t;
accel_ideal = [data(:).u_acc_b];
omega_ideal = [data(:).u_gyr_b];

% Calibrated Accelerometer measurements
T_skew = eye(3);
T_skew(1,2) = -skew_a(3);
T_skew(1,3) = skew_a(2);
T_skew(2,3) = -skew_a(1);
accel_corrupt = inv(diag(scale_a))*inv(T_skew)*C_ma.'*accel_ideal - bias_a ...
                + randn(size(accel_ideal))*sigma_accel;

% Calibrated Gyroscope measurements
T_skew = eye(3);
T_skew(1,2) = -skew_g(3);
T_skew(1,3) = skew_g(2);
T_skew(2,3) = -skew_g(1);
gyro_corrupt = inv(diag(scale_g))*inv(T_skew)*C_mg.'*omega_ideal - bias_g ...
               + randn(size(omega_ideal))*sigma_gyro;

dataIMU.accel= accel_corrupt;
dataIMU.gyro = gyro_corrupt;
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_dot, data] = kinematics(t,x,g_a,pp)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Simulation of the truth
    r_zw_a = x(1:3);
    v_zw_a = x(4:6);
    C_ba = reshape(x(7:15),3,3);
    
    traj = ppval(pp,t);
    trajDerv1 = splineDerv(pp,t,1);
    trajDerv2 = splineDerv(pp,t,2);
    
    a_zw_a = trajDerv2(1:3);
    om_ba_b = quatrate2omega(traj(4:7)./norm(traj(4:7)), trajDerv1(4:7));
    
    r_zw_a_dot = v_zw_a;
    v_zw_a_dot = a_zw_a;
    C_ba_dot = -CrossOperator(om_ba_b)*C_ba;
    x_dot = [r_zw_a_dot;v_zw_a_dot;C_ba_dot(:)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    data.r_zw_a = r_zw_a;
    data.v_zw_a = v_zw_a;
    data.a_zw_a = a_zw_a;
    data.u_acc_b = C_ba*(a_zw_a - g_a);
    data.u_gyr_b = om_ba_b;
    data.C_ba = C_ba;
end
