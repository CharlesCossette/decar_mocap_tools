% An example of how to use the Mocap tools.

dataMocap = mocap_csv2struct('Sensor_Frame_Calibration_Take_002.csv')
spline = mocap_fitSpline(dataMocap)
dataIMU = IMU_csv2struct('2020_07_15_trial2_mmagent1_imu_sensorframe_calibration.csv')
dataSynced = syncTime(spline.RigidBody002, dataIMU)
C_sm = calibrateFrames(dataSynced)