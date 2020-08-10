clear all
% STARTED BUT NOT FINISHED.
%% Generate Truth
r_zw_a_0 = [0;20;0];
v_zw_a_0 = [0;20;0.2];
C_ba_0 = eye(3);
x_0 = [r_zw_a_0;v_zw_a_0; C_ba_0(:)];
tspan = linspace(0,120,120*120);
options = odeset('AbsTol',1e-16,'RelTol',1e-13);
[t,x] = ode45(@(t,x)kinematics(t,x), tspan, x_0, options);
for lv1 = 1:size(x,1)
    [~,data(lv1)] = kinematics(t(lv1),x(lv1,:).');
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
% TODO: Unfortunately there's some discontinuities in the quaternion data
% from simulation! 
dataMocap.RigidBody.q_ba = dcm2quat(C_ba).';
dataMocap.RigidBody.mocapGaps = [];

% For the IMU truth is again sampled but at higher frequency. 
tspan = linspace(0,120,120*236);
options = odeset('AbsTol',1e-16,'RelTol',1e-13);
[t,x] = ode45(@(t,x)kinematics(t,x), tspan, x_0, options);
for lv1 = 1:size(x,1)
    [~,data(lv1)] = kinematics(t(lv1),x(lv1,:).');
end
dataIMU.t = t;
dataIMU.accel = [data(:).u_acc_b] + randn(size([data(:).u_acc_b]))*0.01;
dataIMU.gyro = [data(:).u_gyr_b] + randn(size([data(:).u_gyr_b]))*0.01;


%% Test 
spline = mocap_fitSpline(dataMocap,5,true)
dataSynced = syncTime(spline.RigidBody,dataIMU)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_dot, data] = kinematics(t,x)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Simulation of the truth
    r_zw_a = x(1:3);
    v_zw_a = x(4:6);
    C_ba = reshape(x(7:15),3,3);
    
    a_zw_a = [-20*cos(t);
              -20*sin(t);
                0];
    om_ba_b = [0.5;0.5;0.5];
    
    r_zw_a_dot = v_zw_a;
    v_zw_a_dot = a_zw_a;
    C_ba_dot = -CrossOperator(om_ba_b)*C_ba;
    x_dot = [r_zw_a_dot;v_zw_a_dot;C_ba_dot(:)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    g_a = [0;0;-9.80665];
    data.r_zw_a = r_zw_a;
    data.v_zw_a = v_zw_a;
    data.a_zw_a = C_ba.'*a_zw_a;
    data.u_acc_b = C_ba*(a_zw_a - g_a);
    data.u_gyr_b = om_ba_b;
    data.C_ba = C_ba;
end
