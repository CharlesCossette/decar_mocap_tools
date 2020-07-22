% An example of how to use the Mocap tools.

clear all; close all; clc;

addpath('utils/')
addpath('calibrationData/')

% Extract Mocap data
dataMocap = mocap_csv2struct('Sensor_Frame_Calibration_Take_002.csv')

% Fit a b-spline to the Mocap data
spline = mocap_fitSpline(dataMocap)

% Extract the IMU data
dataIMU = IMU_csv2struct('2020_07_15_trial2_mmagent1_imu_sensorframe_calibration.csv')

% Synchronize the Mocap and IMU data
dataSynced = syncTime(spline.RigidBody002, dataIMU)

%% Manually check axes alignment

% Plot synced data - accelerometers
figure
sgtitle('Accelerometers')
subplot(3,1,1)
plot(dataSynced.t, dataSynced.accMocap(1,:))
hold on
plot(dataSynced.t, dataSynced.accIMU(1,:))
grid
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\ddot{x}$ [$m/s^2$]', 'Interpreter', 'Latex')
legend('Mocap Data', 'IMU Data')
subplot(3,1,2)
plot(dataSynced.t, dataSynced.accMocap(2,:))
hold on
plot(dataSynced.t, dataSynced.accIMU(2,:))
grid
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\ddot{y}$ [$m/s^2$]', 'Interpreter', 'Latex')
subplot(3,1,3)
plot(dataSynced.t, dataSynced.accMocap(3,:))
hold on
plot(dataSynced.t, dataSynced.accIMU(3,:))
grid
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\ddot{z}$ [$m/s^2$]', 'Interpreter', 'Latex')

% Plot synced data - gyroscopes
figure
sgtitle('Gyroscopes')
subplot(3,1,1)
plot(dataSynced.t, dataSynced.omegaMocap(1,:))
hold on
plot(dataSynced.t, dataSynced.omegaIMU(1,:))
grid
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\omega_x$ [rad/$s$]', 'Interpreter', 'Latex')
legend('Mocap Data', 'IMU Data')
subplot(3,1,2)
plot(dataSynced.t, dataSynced.omegaMocap(2,:))
hold on
plot(dataSynced.t, dataSynced.omegaIMU(2,:))
grid
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\omega_y$ [rad/$s$]', 'Interpreter', 'Latex')
subplot(3,1,3)
plot(dataSynced.t, dataSynced.omegaMocap(3,:))
hold on
plot(dataSynced.t, dataSynced.omegaIMU(3,:))
grid
xlabel('$t$ [$s$]', 'Interpreter', 'Latex')
ylabel('$\omega_z$ [rad/$s$]', 'Interpreter', 'Latex')

drawnow

input(strcat('Double check the alignment of the axes from the generated plots.',...
               ' If they are not aligned, please manually align the axes;',...
               ' otherwise, press enter to proceed.'));

% Aligning the axes based on the plots
dataSynced.accIMU(1,:) = -dataSynced.accIMU(1,:);
dataSynced.omegaIMU(2,:) = -dataSynced.omegaIMU(2,:);
dataSynced.omegaIMU(3,:) = -dataSynced.omegaIMU(3,:);

close all

%% Find the DCM between the two assigned body frames

[C_sm, costFuncHist] = calibrateFrames(dataSynced);

