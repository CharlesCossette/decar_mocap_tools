function data = sensorsCsvToStruct(filename)
%SENSORSCSVTOSTRUCT Extracts data from a CSV file, and returns a struct.
%The x,y,z components of vector quantities are lumped into a [3 x N]
%matrix, and all units are converted to SI units.

    % Import raw data
    T = readtable(filename,'PreserveVariableName',true);
    header_names = T.Properties.VariableNames;    
    
    % Find indices of columns containing "timestamp".
    idx_time = find(contains(header_names,'timestamp','IgnoreCase',true));
    
    if numel(idx_time) == 0
        % Then header row was not properly detected.
        % we will try the first row instead
        opts = detectImportOptions(filename,'VariableNamesLine',1,'PreserveVariableNames',true);
        T = readtable(filename,opts);
        header_names = T.Properties.VariableNames;   
        idx_time = find(contains(header_names,'timestamp','IgnoreCase',true));
    end
    
    % If there are multiple timestamp columns, we interpret this as
    % multiple parallel data series concatenated side-by-side.
    if numel(idx_time) > 1
        idx_time_diff = diff(idx_time);
        idx_series_start = idx_time([true,(idx_time_diff > 1)]);
    else
        idx_series_start = idx_time;
    end
    
    data = struct();
    for lv1 = 1:numel(idx_series_start)
        if lv1 == numel(idx_series_start)
            idx_series = idx_series_start(lv1):numel(header_names);
        else
            idx_series = idx_series_start(lv1):(idx_series_start(lv1+1) - 1);
        end
        data_raw = table2struct(rmmissing(T(:,idx_series),'MinNumMissing',numel(idx_series)),'ToScalar',true); 

        header_series = header_names(idx_series);
        data_series = struct();
        data_series = cleanupTimestamps(data_series, data_raw, header_series);
        data_series = cleanupAccel(data_series, data_raw, header_series);
        data_series = cleanupGyro(data_series, data_raw, header_series);
        data_series = cleanupMag(data_series, data_raw, header_series);
        data_series = cleanupPres(data_series, data_raw, header_series);
        data_series = cleanupTemp(data_series, data_raw, header_series);
        data_series = cleanupEuler(data_series, data_raw, header_series);
        data_series = cleanupQuat(data_series, data_raw, header_series);
        %data_series = cleanupUWB(data, data_raw, header_names);
        fieldnames = fields(data_series);
        data_series.t = data_series.(fieldnames{1}); % I hate this. 
        data_series.t_0 = data_series.t(1);
        data_series.t = data_series.t - data_series.t_0;
        data.(['data',num2str(lv1)]) = data_series;
    end
    fieldnames = fields(data);
    if numel(fieldnames) == 1
        data = data.(fieldnames{1});
    end
end
function data = cleanupTimestamps(data, data_raw, header_names)
    data = cleanupScalar(data, data_raw, header_names, 'time');
end
function data = cleanupAccel(data, data_raw, header_names)
    data = cleanup3dVector(data, data_raw, header_names, 'accel');
end

function data = cleanupGyro(data, data_raw, header_names)
    data = cleanup3dVector(data, data_raw, header_names, 'gyro');
end

function data = cleanupMag(data, data_raw, header_names)
    data = cleanup3dVector(data, data_raw, header_names, 'mag');
end

function data = cleanupPres(data, data_raw, header_names)
    data = cleanupScalar(data, data_raw, header_names, 'pressure');
end

function data = cleanupTemp(data, data_raw, header_names)
    data = cleanupScalar(data, data_raw, header_names, 'temp');
end

function data = cleanupEuler(data, data_raw, header_names)
    % TODO: this is untested.
    roll = cleanupScalar(struct(), data_raw, header_names, 'roll');
    pitch = cleanupScalar(struct(), data_raw, header_names, 'pitch');
    yaw = cleanupScalar(struct(), data_raw, header_names, 'yaw');
    % This isnt amazing. If you have multiple sets of euler angles...
    if isfield(roll,'roll') && isfield(pitch,'pitch') && isfield(yaw,'yaw')
        data.euler_angles = [roll.roll(:).'; pitch.pitch(:).'; yaw.yaw(:).'];
    end
end

function data = cleanupQuat(data, data_raw, header_names)
    data = cleanup4dVector(data, data_raw, header_names, 'quat');
end

function data = cleanupScalar(data, data_raw, header_names, identifier)
%CLEANUPSCALAR Searches the header_names for the presence of "identifier"
%after which it extracts the data, converts to SI units, and adds it to the
%struct. In the event that there are multiple columns containing
%"identifier", the name will be modified so that all fields of the output
%struct are unique.

    idx = find(contains(header_names,identifier,'IgnoreCase',true));
    fieldnames = fields(data_raw);
    
    if numel(idx) > 1
        % We have multiple entries containing "identifier".
        warning(['DECAR_MOCAP_TOOLS: multiple [', identifier,'] columns detected.'])
        counter = 1;
        idx_fields = fieldnames(idx);
        for lv1 = 1:numel(idx_fields)
            if isfield(data,idx_fields{lv1})
                % We have repeated column names. Append _counter to the
                % name.
                data.([idx_fields{lv1},'_',num2str(counter)])...
                    = convertToSI(data_raw.(fieldnames{idx(lv1)}), header_names{idx(lv1)});
                counter = counter + 1;
            else
                data.(idx_fields{lv1})...
                    = convertToSI(data_raw.(fieldnames{idx(lv1)}), header_names{idx(lv1)});
            end
        end
    elseif numel(idx) == 1 
        data_temp = data_raw.(fieldnames{idx});
        header = header_names{idx};
        data_temp = convertToSI(data_temp, header);
        data.(identifier) = data_temp;
    end
end

function data = cleanup3dVector(data, data_raw, header_names, identifier)
%CLEANUP3DVECTOR Searches the header_names for the x, y, z components of a
%specific 3D vector quantity with a name containing the "identifier". Then,
%this function converts each component to SI units.

    idx_x = find(contains(header_names,identifier,'IgnoreCase',true)...
                       & (contains(header_names,' x') | contains(header_names,'_x')));
    idx_y = find(contains(header_names,identifier,'IgnoreCase',true)...
                       & (contains(header_names,' y') | contains(header_names,'_y')));
    idx_z = find(contains(header_names,identifier,'IgnoreCase',true)...
                       & (contains(header_names,' z') | contains(header_names,'_z')));

    if numel(idx_x) > 1 || numel(idx_y) >  1 || numel(idx_z) >  1
        error(['DECAR_MOCAP_TOOLS: multiple ', identifier,' detected'])
        % TODO. This will break if we have more than 1 of the same 3d vector
        % measurement.
    elseif numel(idx_x) == 1 && numel(idx_y) ==  1 && numel(idx_z) ==  1 

        fieldnames = fields(data_raw);

        data_x = data_raw.(fieldnames{idx_x});
        header_x = header_names{idx_x};
        data_x = convertToSI(data_x, header_x);

        data_y = data_raw.(fieldnames{idx_y});
        header_y = header_names{idx_y};
        data_y = convertToSI(data_y, header_y);

        data_z = data_raw.(fieldnames{idx_z});
        header_z = header_names{idx_z};
        data_z = convertToSI(data_z, header_z);

        data.(identifier) = [data_x(:).'; data_y(:).'; data_z(:).'];
    end
end

function data = cleanup4dVector(data, data_raw, header_names, identifier)
%CLEANUP3DVECTOR Searches the header_names for the w, x, y, z components of a
%specific 4D vector quantity with a name containing the "identifier". Then,
%this function converts each component to SI units. Currently this only
%applies to quaternions.
    idx_w = find(contains(header_names,identifier,'IgnoreCase',true)...
                       & (contains(header_names,' w') | contains(header_names,'_w')));
    idx_x = find(contains(header_names,identifier,'IgnoreCase',true)...
                       & (contains(header_names,' x') | contains(header_names,'_x')));
    idx_y = find(contains(header_names,identifier,'IgnoreCase',true)...
                       & (contains(header_names,' y') | contains(header_names,'_y')));
    idx_z = find(contains(header_names,identifier,'IgnoreCase',true)...
                       & (contains(header_names,' z') | contains(header_names,'_z')));

    if numel(idx_x) > 1 || numel(idx_y) >  1 || numel(idx_z) >  1 || numel(idx_w) > 1
        error(['DECAR_MOCAP_TOOLS: multiple ', identifier,' detected'])
        % TODO. This will break if we have more than 1 of the same 4d vector
        % measurement.
    elseif numel(idx_x) == 1 && numel(idx_y) ==  1 && numel(idx_z) ==  1 && numel(idx_w) == 1

        fieldnames = fields(data_raw);

        data_w = data_raw.(fieldnames{idx_w});
        header_w = header_names{idx_w};
        data_w = convertToSI(data_w, header_w);

        data_x = data_raw.(fieldnames{idx_x});
        header_x = header_names{idx_x};
        data_x = convertToSI(data_x, header_x);

        data_y = data_raw.(fieldnames{idx_y});
        header_y = header_names{idx_y};
        data_y = convertToSI(data_y, header_y);

        data_z = data_raw.(fieldnames{idx_z});
        header_z = header_names{idx_z};
        data_z = convertToSI(data_z, header_z);

        data.(identifier) = [data_w(:).'; data_x(:).'; data_y(:).'; data_z(:).'];
    end
end
    
function [data_si, header_unitless] = convertToSI(data_raw, header)
    unit = '';
    if contains(header,'(g)') % Acceleration units
        unit = '(g)';
        data_si = 9.80665*data_raw;
    elseif contains(header,'(mg)')
        unit = '(mg)';
        data_si = (9.80665/1000)*data_raw;
    elseif contains(header,'(m/s^2)')
        unit = '(m/s^2)';
        data_si = data_raw;
    elseif contains(header,'(mm/s^2)')
        unit = '(mm/s^2)';
        data_si = (1/1000)*data_raw;
    elseif contains(header,'(deg/s)') % Angular velocity
        data_si = (2*pi/360)*data_raw;
    elseif contains(header,'(rad/s)')
        data_si = data_raw;
    elseif contains(header,'(mrad/s)')
        data_si = (1/1000)*data_raw;
    elseif contains(header,'(uT)') % Magnetic field strength
        data_si = (1e-6)*data_raw;
    elseif contains(header,'(mT)')
        data_si = (1e-3)*data_raw;
    elseif contains(header,'(gauss)')
        data_si = (1e-4)*data_raw;
    elseif contains(header,'(mgauss)')
        data_si = (1e-4)*(1/1000)*data_raw;
    elseif contains(header,'(G)')
        data_si = (1e-4)*data_raw;
    elseif contains(header,'(T)')
        data_si = data_raw;
    elseif contains(header,'(bar)') % Pressure
        data_si = 100000*data_raw;
    elseif contains(header,'(mbar)')
        data_si = 100*data_raw;
    elseif contains(header,'(MPa)')
        data_si = 1000000*data_raw;
    elseif contains(header,'(hPa)')
        data_si = 100*data_raw;
    elseif contains(header,'(Pa)')
        data_si = data_raw;
    elseif contains(header,'(F)') % Temperature
        data_si = (data_raw - 32)*(5/9);
    elseif contains(header,'(K)')
        data_si = data_raw - 273;
    elseif contains(header,'(C)')
        data_si = data_raw;
    elseif contains(header,'(km)') % Distances
        data_si = 1000*data_raw;
    elseif contains(header,'(cm)')
        data_si = 1e-2*data_raw;
    elseif contains(header,'(mm)')
        data_si = 1e-3*data_raw;
    elseif contains(header,'(m)') 
        data_si = data_raw;
    elseif contains(header,'(deg)') % Angles
        data_si = (2*pi/360)*data_raw;
    elseif contains(header,'(rad)') 
        data_si = data_raw;
    elseif contains(header,'(ns)')  % Time
        unit = '(ns)';
        data_si = (1e-9)*data_raw;
    elseif contains(header,'(us)') 
        unit = '(us)';
        data_si = (1e-6)*data_raw;
    elseif contains(header,'(ms)') 
        unit = '(ms)';
        data_si = (1e-3)*data_raw;
    elseif contains(header,'(s)') 
        unit = '(s)';
        data_si = data_raw;   
    else
        warning(['DECAR_MOCAP_TOOLS: Unit missing or unsupported for ',header]);
        data_si = data_raw;
    end
    header_unitless = erase(header,unit);
end
    
