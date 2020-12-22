function S = UWB_csv2struct(filename, t_0)
% UWB_CSV2STRUCT [DEPRECATED]
% extracts the range information from the csv file generated
% by a RPI or a Pozyx device, and stores it into a struct. 

% WARNING: Requires MATLAB 2019a or later with the AEROSPACE TOOLBOX.

% Inputs:
% --------
% filename: [string]
%       filename of Range raw csv, must be on the matlab path.
% t0: [double]
%       Time to subtract from all the timestamps.
%       the value used to subtract the timestamps of the IMU data collected
%       on the same device as the range measurements. This allows
%       synchronization between the UWB data and the IMU data. 
%
% Outputs:
% --------
%   S: [struct] with fields
%       t: [N x 1] double.
%           Time points of all the data.
%       tags_0x####_0x####: [N x 1] double
%           Range measurements between the tags specified in the field
%           name.
    
S = uwbCsvToStruct(filename, t_0);

