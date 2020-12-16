function data_pivot = mocapSetNewPivotPoint(data_mocap, pos_shift, varargin)
%MOCAPSETNEWPIVOTPOINT Modifies the position data inside a data_mocap
%struct to be the position of a new "origin", "pivot point", or, as our
%group calls it, "point z". i.e. a reference point on the body.
%
% PARAMETERS:
% -----------
% data_mocap: struct
%       Mocap data. Could be in the format extracted by mocapCsvToStruct() or
%       by imuMocapSyncTime().
% pos_shift: [3 x 1] double
%       Position shift IN THE BODY FRAME of <rigid_body_name>. I.e. this
%       quantity should be r_pz_b, where p will be the new reference point.
% varargin: name-value pairs
%   'rigid_body_name'
%       Default: 'None'
%       Provide a string of the name of the target rigid body for the
%       modification, as named by the mocap, only if data_mocap is the output of
%       mocapCsvToStruct().
%   'fit_spline_bool'
%       Default: false
%       Provide a true/false value on whether the user would like to fit a
%       spline to the mocap data after the modification. If true, corresponding
%       velocity, acceleration, and ground truth accel/gyro data are extracted.
%   'gap_size'
%       Default: 1
%       Provide a float for the downsampling of data for quick spline fitting.
%   'g_a'
%       Default: [0;0;-9.80665]
%       Provide a [3 x 1] double representing the calibrated gravity vector.
%
% RETURNS:
% --------
% data_pivot: struct
%       Supplied mocap data but with the modifications.

    if nargin < 2
        error('Missing data')
    end
    
    % Default options for optional arguments
    default_rigid_body_name = 'None';
    default_fit_spline_bool = false;
    default_gap_size = 1;
    default_g_a = [0;0;-9.80665];
    
    % Parse input for name-value pairs
    p = inputParser;
    addRequired(p,'data_mocap');
    addRequired(p,'pos_shift');
    addParameter(p,'rigid_body_name',default_rigid_body_name);
    addParameter(p,'fit_spline_bool', default_fit_spline_bool);
    addParameter(p,'gap_size', default_gap_size);
    addParameter(p,'g_a', default_g_a);
    
    % Load input into variables.
    parse(p, data_mocap, pos_shift, varargin{:})
    data_mocap = p.Results.data_mocap;
    pos_shift = p.Results.pos_shift;
    rigid_body_name = p.Results.rigid_body_name;
    fit_spline_bool = p.Results.fit_spline_bool;
    gap_size = p.Results.gap_size;
    g_a = p.Results.g_a;
    
    % Extarct the data associated with the rigid body being modified
    if strcmp(rigid_body_name,'None')
        data_rigid_body = data_mocap;
    else
        data_rigid_body = data_mocap.(rigid_body_name);
    end
    
    % Move the mocap pivot point
    for lv1=1:1:length(data_mocap.t)
        data_rigid_body.r_zw_a(:,lv1) ...
                = data_rigid_body.r_zw_a(:,lv1)...
                    + data_rigid_body.C_ba(:,:,lv1)'*pos_shift;
    end
       
    % Fit a spline to the modified mocap data
    if fit_spline_bool
        if isfield(data_rigid_body, 'q_ba')
            % Extract the spline parameters
            data_rigid_body.type = 'Rigid Body';
            data_rigid_body.gapIntervals = mocapGetGapIntervals(data_rigid_body);
            mocap_spline = mocapGetSplineProperties(data_rigid_body, gap_size);
            
            % Extract the ground truth accel/gyro measurements, and correct the
            % velocity and acceleration data using the new pivot point
            [mocap_accel, mocap_gyro, mocap_corrected]...
                    = getFakeImuMocap(mocap_spline,data_rigid_body.t,g_a);
        else
            error('Quaternion data is required to fit a spline using MOCAP Tools.')
        end
    end
    
    % Save the modified data to the output
    data_pivot = data_mocap;
    if strcmp(rigid_body_name,'None')
        data_pivot.r_zw_a = data_rigid_body.r_zw_a;
        if fit_spline_bool
            data_pivot.v_zwa_a = mocap_corrected.v_zwa_a;
            data_pivot.a_zwa_a = mocap_corrected.a_zwa_a;
            data_pivot.accel_mocap = mocap_accel;
            data_pivot.gyro_mocap = mocap_gyro;
        end
    else
        data_pivot.(rigid_body_name).r_zw_a = data_rigid_body.r_zw_a;
        if fit_spline_bool
            data_pivot.(rigid_body_name).v_zwa_a = mocap_corrected.v_zwa_a;
            data_pivot.(rigid_body_name).a_zwa_a = mocap_corrected.a_zwa_a;
            data_pivot.(rigid_body_name).accel_mocap = mocap_accel;
            data_pivot.(rigid_body_name).gyro_mocap = mocap_gyro;
        end
    end
    
end