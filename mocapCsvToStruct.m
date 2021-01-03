function data_mocap = mocapCsvToStruct(filename, mocap_system)
% mocapCsvToStruct calls the function corresponding to the Motion capture 
% system specified by the user. This then extracts the relevant information 
% from the raw csv file and stores it into a struct. 
%
% If the OptiTrack system is used, mocapCsvToStruct realigns the Mocap frame to
% to have the z-axis pointing upwards.
%
% PARAMETERS:
% -----------
% filename: [string]
%       filename of Optitrack raw csv, must be on the matlab path.
% mocap_system: [string]
%       Default: 'optitrack'
%       The Mocap system used. Currently supported options are OptiTrack and 
%       Vicon.
% RETURNS:
% ---------
%   data_mocap: [struct] with fields
%       t: [N x 1] double.
%           Time points of all the data.
%       r_zw_a: [3 x N] double.
%           Location of "pivot point" of the rigid body, or location of the
%           marker in the mocap world frame set by calibrating the ground
%           plane.
%       q_ba: [4 x N] double.
%           Attitude quaternion provided by the Optitrack system, which has
%           been corrected to correspond to the new "a" frame after the
%           axis switch described above.
%       C_ba: [3 x 3 x N] double.
%           DCM corresponding to the above quaternion.
%       gapIntervals: [2 x M]
%           Sections of time where no ground truth data was collected.
%       staticIntervals: [2 x K]
%           Sections of time where the rigid body was static.

    if nargin < 2
        mocap_system = 'optitrack';
    end

    switch lower(mocap_system)
        case 'optitrack'
            data_mocap = optitrackCsvToStruct(filename);
        case 'vicon'
            data_mocap = viconCsvToStruct(filename);
        otherwise
            error('DECAR_MOCAP_TOOLS: Mocap system unsupported.')
    end

end