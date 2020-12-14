function data_uwb_corrected = calibrateUWB(spline_mocap, data_uwb, r_pz_b,...
                                           body_names, tag_names, remove_outliers)
% calibrateUWB corrects the UWB measurements by removing biases using 
% ground truth data. To do so, a heavily smoothed spline is fit to the
% error between the range measurements and the ground truth distance, which
% is then subtracted from the range measurements.

% Inputs:
% --------
% spline_mocap: [struct]
%       Contains a bunch of spline parameters of the different rigid bodies
%       detected by the Mocap system. Used to extract the ground truth
%       distance between the tags.
% data_uwb: [struct]
%       Contains the range measurements between each pair of tags.
% r_pz_b: [3 x M] double
%       An array where each column represents the position of a tag
%       relative to the reference point of the Mocap system.
% body_names: [1 x M] cell
%       Contains the RigidBody names where each tag lies, in the same tag
%       order specified by r_pz_b. The names come from the convention used
%       by the Mocap system.
% tag_names: [1 x M] cell
%       Contains the serial code of each tag, in the same order specified
%       by r_pz_b. 
% remove_outliers: boolean
%       A boolean input to specify whether or not the output data should include
%       outliers.

% Outputs:
% --------
%   data_uwb_corrected: [struct] with fields
%       tags_0x####_0x####: [struct] with fields
%           t: [N x 1] double.
%              Time points in which measurements were collected between
%              these two tags.
%           meas: [N x 1] double.
%              The measurements between the two tags.
%
data_uwb_corrected = uwbCalibrate(spline_mocap, data_uwb, r_pz_b,...
                                           body_names, tag_names, remove_outliers);