function  S = IMU_csv2struct(filename)
% Extracts data from one IMU.

    opts = detectImportOptions(filename);
    header_row_range = [num2str(1),':',num2str(opts.DataLine(1))+1];

    headers = readcell(filename,'Range',header_row_range);
    data = readmatrix(filename, 'NumHeaderLines',opts.DataLine(1));

    %% STEP 1 - Remove any columns with barely any data
    % Criteria: If less than 10% of rows have data, delete column.
    to_delete = false(1,size(data,2));
    for lv1 = 1:size(data,2)
        data_col = data(:,lv1);
        indx = logical(isnan(data_col) + (data_col == 0));
        if (sum(indx)/size(data,1)) > 0.3
            to_delete(lv1) = true;
        end
    end

    headers = headers(:,~to_delete);
    data = data(:,~to_delete);

    %% Step 2 - Get accelerometer, gyroscope, magnetometer, and barometer data
    % Find the row where the main headers are present
    headerRow = 1;
    while 1
        if headerRow > 100
            break
        end
        try 
            if ~isempty(find(contains(headers(headerRow,:),'Timestamp')))
                break
            else
                headerRow = headerRow + 1;
            end
        catch
            headerRow = headerRow + 1;
        end
    end

    % Timestep
    index = find(contains(headers(headerRow,:),'Timestamp'));
    S.t = data(:,index);
    S.t = S.t - S.t(1); % Reset the time to start from 0
    % ensure unit is seconds
    if contains(headers(headerRow,index),'(ns)') 
        S.t = S.t./10^9;        % ns to s
    elseif ~contains(headers(headerRow,index),'(s)')
        warning('Unrecognized time unit.')
    end

    % Accelerometer
    indices = find(contains(headers(headerRow,:),'Accel'));
    S.accel = [data(:,indices(1)), data(:,indices(2)), data(:,indices(3))]';
    % ensure acceleration is in m/s^2
    if contains(headers(headerRow,indices(1)),'(g)') 
        S.accel = S.accel.*9.81;        % g to m/s^2
    elseif contains(headers(headerRow,indices(1)),'(mg)') 
        S.accel = S.accel./1000.*9.81;  % mg to m/s^2
    elseif ~contains(headers(headerRow,indices(1)),'(m/s^2)') && ~contains(headers(headerRow,indices(1)),'(m/s2)')
        warning('Unrecognized acceleration unit.')
    end

    % Gyroscope
    indices = find(contains(headers(headerRow,:),'Gyro'));
    S.gyro = [data(:,indices(1)), data(:,indices(2)), data(:,indices(3))]';
    % ensure gyroscope reading is in rad/s
    if contains(headers(headerRow,indices(1)),'(deg/s)') 
        S.gyro = S.gyro.*(pi/180);        % deg/s to rad/s
    elseif ~contains(headers(headerRow,indices(1)),'(rad/s)')
        warning('Unrecognized gyroscope unit.')
    end

    % Magnetometer
    indices = find(contains(headers(headerRow,:),'Mag'));
    S.mag = [data(:,indices(1)), data(:,indices(2)), data(:,indices(3))]';
    % ensure magnetometer reading is in uT
    if ~contains(headers(headerRow,indices(1)),'(uT)') && ~contains(headers(headerRow,indices(1)),'(muT)')
        warning('Unrecognized magnetometer unit.')
    end

    % Barometer
    % TODO: 1) deal with mBar too..
    index = find(contains(headers(headerRow,:),'Pressure'));
    S.bar = data(:,index);
    % ensure barometer reading is in Pa
    if ~contains(headers(headerRow,index),'(Pa)')
        warning('Unrecognized barometer unit.')
    end

end

