%% Test 1 - Dead reckoning
clear;
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate imu/mocap data.
params.C_ms_accel = eye(3);
params.C_ms_gyro = eye(3);
params.bias_accel = [0;0;0];
params.bias_gyro = [0;0;0];
params.scale_accel = [1;1;1];
params.scale_gyro = [1;1;1];
params.skew_accel = [0;0;0];
params.skew_gyro = [0;0;0];
params.mocap_gravity = [0;0;-9.80665];
params.std_dev_accel = 0;
params.std_dev_gyro = 0;
params.mocap_frequency = 120;
params.imu_frequency = 250;
params.sim_duration = 40;
[dataMocap, dataIMU] = simulateTestData(params);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%               
t_span = dataIMU.t;
r_zw_a_0 = dataMocap.RigidBody.r_zw_a(:,1); 
v_zwa_a_0 = [0;0;0];
C_ba_0 = dataMocap.RigidBody.C_ba(:,:,1);
g_a = params.mocap_gravity;

traj_rk4 = imuDeadReckoning(dataIMU, r_zw_a_0, v_zwa_a_0, C_ba_0, g_a, 'rk4',t_span);
traj_euler = imuDeadReckoning(dataIMU, r_zw_a_0, v_zwa_a_0, C_ba_0, g_a, 'euler',t_span);
traj_so3 = imuDeadReckoning(dataIMU, r_zw_a_0, v_zwa_a_0, C_ba_0, g_a, 'so3',t_span);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: Add some asserts

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compare results visually
% Position
close all
figure(1)
r_zw_a_rk4 = traj_rk4.r_zw_a;
r_zw_a_euler = traj_euler.r_zw_a;
r_zw_a_so3 = traj_so3.r_zw_a;
r_zw_a_gt = dataMocap.RigidBody.r_zw_a;
plot3(r_zw_a_gt(1,:), r_zw_a_gt(2,:), r_zw_a_gt(3,:),'LineWidth',2)
hold on
plot3(r_zw_a_so3(1,:), r_zw_a_so3(2,:), r_zw_a_so3(3,:))
plot3(r_zw_a_rk4(1,:), r_zw_a_rk4(2,:), r_zw_a_rk4(3,:),'LineWidth',2)
plot3(r_zw_a_euler(1,:), r_zw_a_euler(2,:), r_zw_a_euler(3,:))
legend('Ground Truth','SO(3) Euler','RK4','Quaternion Euler')
title('Position Dead-Reckoning Results')

hold off
axis vis3d
axis equal
grid on
%axis([-20 20 -20 20 -20 20])

% Attitude 
figure(3)
phi_ba_gt = DCM_TO_ROTVEC(dataMocap.RigidBody.C_ba);
phi_ba_so3 = DCM_TO_ROTVEC(traj_so3.C_ba);
phi_ba_rk4 = DCM_TO_ROTVEC(quat2dcm(traj_rk4.q_ba.'));
phi_ba_euler = DCM_TO_ROTVEC(quat2dcm(traj_euler.q_ba.'));

subplot(3,1,1)
plot(dataMocap.RigidBody.t,phi_ba_gt(1,:))
hold on
plot(dataIMU.t, phi_ba_so3(1,:))
plot(dataIMU.t, phi_ba_rk4(1,:))
plot(dataIMU.t, phi_ba_euler(1,:))
hold off
grid on
title('Attitude Dead-Reckoning Results')
legend('Ground Truth','SO(3) Euler','RK4','Quaternion Euler')

subplot(3,1,2)
plot(dataMocap.RigidBody.t,phi_ba_gt(2,:))
hold on
plot(dataIMU.t, phi_ba_so3(2,:))
plot(dataIMU.t, phi_ba_rk4(2,:))
plot(dataIMU.t, phi_ba_euler(2,:))
hold off
grid on

subplot(3,1,3)
plot(dataMocap.RigidBody.t,phi_ba_gt(3,:))
hold on
plot(dataIMU.t, phi_ba_so3(3,:))
plot(dataIMU.t, phi_ba_rk4(3,:))
plot(dataIMU.t, phi_ba_euler(3,:))
hold off
grid on


