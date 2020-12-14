function  S = mocap_csv2struct(filename)
%MOCAP_CSV2STRUCT [DEPRECATED]
%Extracts the relevant information from the raw Optitrack
%csv file and stores it into a struct. Also removes any markers with a
%visibility less than 10%, and REALIGNS THE AXES SO THAT Z POINTS UP.
%
% The axis switch is as follows:
%   Mocap X becomes Y
%   Mocap Y becomes Z
%   Mocap Z becomes X
%
% If frame b is the mocap body frame, frame a is the mocap "world" frame,
% and frame a' is the new desired world frame where Z points up, we must
% apply the following transformation
%   C_ba' = C_ba * C_aa'
% where
%   C_aa' = [0 1 0; 0 0 1; 1 0 0].
% This transformation is equivalently done below in quaternions with
%   q_aa' = [0.5 0.5 0.5 0.5].
% We use the SCALAR FIRST convention for quaternions.
%
% WARNING: Requires MATLAB 2019a or later with the AEROSPACE TOOLBOX.
%
% Inputs:
% --------
% filename: [string]
%       filename of Optitrack raw csv, must be on the matlab path.
%
% Outputs:
% --------
%   S: [struct] with fields
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
%       mocapGaps: [2 x M]
%           Sections of time where no ground truth data was collected.

S = mocapCsvToStruct(filename)
