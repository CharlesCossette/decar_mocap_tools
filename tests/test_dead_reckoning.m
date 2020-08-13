%% Test 1 - Dead reckoning
clear;
close all

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
params.mocap_frequency = 1000;
params.imu_frequency = 250;

[dataMocap, dataIMU] = simulateTestData(params);


f = @(t,x) imuDeadReckoningODE(t,x,dataIMU.t, dataIMU.accel, ...
                               dataIMU.gyro, params.mocap_gravity);
%%                     
t_span = dataIMU.t;
C_ba_0 = eye(3);
q_ba_0 = dcm2quat(dataMocap.RigidBody.C_ba(:,:,1));
x_0 = [dataMocap.RigidBody.r_zw_a(:,1); 0;0;0;q_ba_0.'];
options.indices_to_normalize = 7:10;

[t, x_rk4] = ode4(f, t_span, x_0, options);
[t, x_euler] = ode1(f, t_span, x_0, options);

%%
N = length(dataIMU.t);
r_zw_a_so3 = zeros(3,N);
v_zwa_a_so3 = zeros(3,N);
C_ba_so3 = zeros(3,3,N);
r_zw_a_so3(:,1) = dataMocap.RigidBody.r_zw_a(:,1);
C_ba_so3(:,:,1) = dataMocap.RigidBody.C_ba(:,:,1);
g_a = params.mocap_gravity;
a_zwa_a_so3 = zeros(3,N);
for lv1 = 1:N-1
    dt = dataIMU.t(lv1+1) - dataIMU.t(lv1);
    omega_ba_b = dataIMU.gyro(:,lv1);
    a_zwa_b = dataIMU.accel(:,lv1);
    C_ba_so3(:,:,lv1+1) = expm(-CrossOperator(omega_ba_b*dt))*C_ba_so3(:,:,lv1);
    v_zwa_a_so3(:,lv1+1) = v_zwa_a_so3(:,lv1) + (C_ba_so3(:,:,lv1).'*a_zwa_b + g_a)*dt;
    r_zw_a_so3(:,lv1+1) = r_zw_a_so3(:,lv1) + v_zwa_a_so3(:,lv1)*dt;
    a_zwa_a_so3(:,lv1) = C_ba_so3(:,:,lv1).'*a_zwa_b + g_a;
end

close all
figure(1)
r_zw_a_rk4 = x_rk4(:,1:3).';
r_zw_a_euler = x_euler(:,1:3).';
r_zw_a_gt = dataMocap.RigidBody.r_zw_a;
plot3(r_zw_a_gt(1,:), r_zw_a_gt(2,:), r_zw_a_gt(3,:),'LineWidth',2)
hold on
plot3(r_zw_a_rk4(1,:), r_zw_a_rk4(2,:), r_zw_a_rk4(3,:),'LineWidth',2)
plot3(r_zw_a_euler(1,:), r_zw_a_euler(2,:), r_zw_a_euler(3,:))
plot3(r_zw_a_so3(1,:), r_zw_a_so3(2,:), r_zw_a_so3(3,:))
hold off
axis vis3d
axis equal
grid on
%axis([-20 20 -20 20 -20 20])

figure(3)
phi_ba_gt = DCM_TO_ROTVEC(dataMocap.RigidBody.C_ba);

phi_ba_so3 = DCM_TO_ROTVEC(C_ba_so3);
phi_ba_rk4 = DCM_TO_ROTVEC(quat2dcm(x_rk4(:,7:10)));
phi_ba_euler = DCM_TO_ROTVEC(quat2dcm(x_euler(:,7:10)));

subplot(3,1,1)
plot(dataMocap.RigidBody.t,phi_ba_gt(1,:))
hold on
plot(dataIMU.t, phi_ba_so3(1,:))
plot(t_span, phi_ba_rk4(1,:))
plot(t_span, phi_ba_euler(1,:))
hold off

subplot(3,1,2)
plot(dataMocap.RigidBody.t,phi_ba_gt(2,:))
hold on
plot(dataIMU.t, phi_ba_so3(2,:))
plot(t_span, phi_ba_rk4(2,:))
plot(t_span, phi_ba_euler(2,:))
hold off

subplot(3,1,3)
plot(dataMocap.RigidBody.t,phi_ba_gt(3,:))
hold on
plot(dataIMU.t, phi_ba_so3(3,:))
plot(t_span, phi_ba_rk4(3,:))
plot(t_span, phi_ba_euler(3,:))
hold off


