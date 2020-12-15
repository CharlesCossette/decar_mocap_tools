function data_pivot = mocapSetNewPivotPoint(data_mocap, pos_shift, varargin)
%MOCAPSETNEWPIVOTPOINT Modifies the position data inside a data_mocap
%struct to be the position of a new "origin", "pivot point", or, as our
%group calls it, "point z". i.e. a reference point on the body.
%
% PARAMETERS:
% -----------
% data_mocap: struct
%       Mocap data as extracted by mocapCsvToStruct()
% rigid_body_name: string
%       target rigid body to apply to modification, as named by the mocap.
% pos_shift: [3 x 1] double
%       Position shift IN THE BODY FRAME of <rigid_body_name>. I.e. this
%       quantity should be r_pz_b, where p will be the new reference point.
% 
% RETURNS:
% --------
% data_mocap: struct
%       Supplied mocap data but with the modification.

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
    
    
    if strcmp(rigid_body_name,'None')
        data_rigid_body = data_mocap;
    else
        data_rigid_body = data_mocap.(rigid_body_name);
    end
    
    for lv1=1:1:length(data_mocap.t)
        data_rigid_body.r_zw_a(:,lv1) ...
                = data_rigid_body.r_zw_a(:,lv1)...
                    + data_rigid_body.C_ba(:,:,lv1)'*pos_shift;
    end
       
    if fit_spline_bool
        data_rigid_body.type = 'Rigid Body';
        data_rigid_body.gapIntervals = mocapGetGapIntervals(data_rigid_body);
        mocap_spline = mocapGetSplineProperties(data_rigid_body, gap_size);
        [mocap_accel, mocap_gyro, mocap_corrected]...
                = getFakeImuMocap(mocap_spline,data_rigid_body.t,g_a);
    end
    
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