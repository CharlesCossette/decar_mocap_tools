clear; close all;

phi = [0.1;0.1;-0.2];
bias_a = [-0.1;0.2;0.3];
bias_g = -[-0.1;0.2;0.3];
C_ma = ROTVEC_TO_DCM(phi);
C_mg = ROTVEC_TO_DCM(-phi);
scale_a = [1.2;1.2;1.1];
scale_g = [1.05;0.95;1];
skew_a = [0;0;0];
skew_g = [0;0;0];
g_a = [0;0;-1];
g_a = 9.80665*(g_a./norm(g_a));

%% Generate Truth
r_zw_a_0 = [0;20;0];
v_zw_a_0 = [0.2;20;0];
C_ba_0 = eye(3);
x_0 = [r_zw_a_0;v_zw_a_0; C_ba_0(:)];
tspan = linspace(0,120,120*120);
options = odeset('AbsTol',1e-16,'RelTol',1e-13);
[t,x] = ode45(@(t,x)kinematics(t,x,g_a), tspan, x_0, options);
clear data
for lv1 = 1:size(x,1)
    [~,data(lv1)] = kinematics(t(lv1),x(lv1,:).',g_a);
end

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

%% Generate Fake Data from truth
dataMocap.RigidBody.r_zw_a = r_zw_a;
dataMocap.RigidBody.t = t;
dataMocap.RigidBody.type = 'Rigid Body';
dataMocap.RigidBody.C_ba = C_ba;
dataMocap.RigidBody.q_ba = smoothdcm2quat(C_ba).';
dataMocap.RigidBody.gapIntervals = [];
dataMocap.RigidBody.staticIntervals = [];

% For the IMU truth is again sampled but at higher frequency. 
tspan = linspace(0,120,120*236);
options = odeset('AbsTol',1e-16,'RelTol',1e-13);
[t,x] = ode45(@(t,x)kinematics(t,x,g_a), tspan, x_0, options);
clear data
for lv1 = 1:size(x,1)
    [~,data(lv1)] = kinematics(t(lv1),x(lv1,:).',g_a);
end
dataIMU.t = t;
accel_ideal = [data(:).u_acc_b]
omega_ideal = [data(:).u_gyr_b]

% Calibrated Accelerometer measurements
T_skew = eye(3);
T_skew(1,2) = -skew_a(3);
T_skew(1,3) = skew_a(2);
T_skew(2,3) = -skew_a(1);
accIMU = inv(diag(scale_a))*inv(T_skew)*C_ma.'*accel_ideal - bias_a;

% Calibrated Gyroscope measurements
T_skew = eye(3);
T_skew(1,2) = -skew_g(3);
T_skew(1,3) = skew_g(2);
T_skew(2,3) = -skew_g(1);
omegaIMU = inv(diag(scale_g))*inv(T_skew)*C_mg.'*omega_ideal - bias_g;

dataIMU.accel= accIMU;
dataIMU.gyro = omegaIMU;
%% Test 
spline = mocap_fitSpline(dataMocap,5,true)
dataSynced = syncTime(spline.RigidBody,dataIMU)
dataSynced.gapIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.gapIntervals);
dataSynced.staticIndices = getIndicesFromIntervals(dataSynced.t, dataMocap.RigidBody.staticIntervals);
%dataAligned = alignFrames(dataSynced)
[results, dataCalibrated] = calibrateFrames(dataSynced)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_dot, data] = kinematics(t,x,g_a)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Simulation of the truth
    r_zw_a = x(1:3);
    v_zw_a = x(4:6);
    C_ba = reshape(x(7:15),3,3);
    
    a_zw_a = [0;
              -20*sin(t);
                -20*cos(t)];
    om_ba_b = [0.5*sin(t) + 0.3*sin(2*t)+0.2;0.5*cos(3*t);0.5*sin(0.5*t)*sin(2*t)-0.5];
    
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
