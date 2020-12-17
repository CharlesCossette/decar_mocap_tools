function S = mocapCsvToStruct(filename, mocap_system)

    if nargin < 2
        mocap_system = 'optitrack';
    end

    switch lower(mocap_system)
        case 'optitrack'
            S = optitrackCsvToStruct(filename);
        case 'vicon'
            S = viconCsvToStruct(filename);
        otherwise
            error('Mocap system unsupported.')
    end

end