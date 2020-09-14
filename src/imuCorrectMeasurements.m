function data_calibrated = imuCorrectMeasurements(data_imu, calib_params)
    data_calibrated = data_imu;
    
    C_ma = calib_params.C_ms_accel;
    C_mg = calib_params.C_ms_gyro;
    bias_a = calib_params.bias_accel;
    bias_g = calib_params.bias_gyro;
    scale_a = calib_params.scale_accel;
    scale_g = calib_params.scale_gyro;
    skew_a = calib_params.skew_accel;
    skew_g = calib_params.skew_gyro;    
    
    % Calibrated Accelerometer measurements
    T_skew_a = eye(3);
    T_skew_a(1,2) = -skew_a(3);
    T_skew_a(1,3) =  skew_a(2);
    T_skew_a(2,3) = -skew_a(1);
    accel_calibrated = C_ma*T_skew_a*diag(scale_a)*(data_imu.accel + bias_a);

    % Calibrated Gyroscope measurements
    T_skew_g = eye(3);
    T_skew_g(1,2) = -skew_g(3);
    T_skew_g(1,3) =  skew_g(2);
    T_skew_g(2,3) = -skew_g(1);
    gyro_calibrated = C_mg*T_skew_g*diag(scale_g)*(data_imu.gyro + bias_g);
    
    data_calibrated.accel = accel_calibrated;
    data_calibrated.gyro = gyro_calibrated;
end