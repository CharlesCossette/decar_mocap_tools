function  S = viconCsvToStruct(filename)
% viconCsvToStruct extracts the relevant information from the raw Vicon
% csv file and stores it into a struct. Also removes any markers with a
% visibility less than 10%.
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
%           Attitude quaternion provided by the Vicon system, which has
%           been corrected to correspond to the new "a" frame after the
%           axis switch described above.
%       C_ba: [3 x 3 x N] double.
%           DCM corresponding to the above quaternion.
%       gapIntervals: [2 x M]
%           Sections of time where no ground truth data was collected.
%       staticIntervals: [2 x K]
%           Sections of time where the rigid body was static.

% Use built-in matlab function to automatically detect the header rows.
opts = detectImportOptions(filename);
header_row_range = [num2str(2),':',num2str(4)];

% Store headers into cell array, data into matrix
headers = readcell(filename,'Range',header_row_range); % MATLAB 2019a +
data = readmatrix(filename, 'NumHeaderLines',opts.DataLine(1));

%% STEP 1 - Remove any columns with barely any data
% Criteria: If less than 10% of rows have data, delete column.
to_delete = false(1,size(data,2));
for lv1 = 1:size(data,2)
    data_col = data(:,lv1);
    indx = logical(isnan(data_col) + (data_col == 0));
    if (sum(indx)/size(data,1)) > 0.1
        to_delete(lv1) = true;
    end
end

headers = headers(:,~to_delete);
data = data(:,~to_delete);


%% STEP 2 - Create the timestamps
% Rewrite the frame numbers as timestamps
frequency = headers{1,1};
data(:,1) = data(:,1) ./ frequency;

%% STEP 3 - Extract the name, position, and attitude of each rigid body
% Find the first column of each rigid body
is_RX = cellfun(@(x) stringincell(x,'RX'),headers);
[~,col_RX] = find(is_RX);

% Save the name, position, and quaternion data to a struct S
if ~isempty(col_RX)
    for lv1=1:numel(col_RX)
        % Extract the name
        name = headers{2, col_RX(lv1)};
        name = strrep(name,'Global Angle (Quaternion) ', '');
        name = strrep(name,':', '');
        name = name(1:length(name)/2);
        
        % Save the timestamps and type to the struct 
        S.(name).t = data(:,1);
        S.(name).type = 'Rigid Body'; % Vicon only records bodies
        
        % Extract attitude measurements
        quat_x = data(:,col_RX(lv1));
        quat_y = data(:,col_RX(lv1)+1);
        quat_z = data(:,col_RX(lv1)+2);
        quat_w = data(:,col_RX(lv1)+3);
        
        S.(name).q_ba = [quat_w.';
                         quat_x.';
                         quat_y.';
                         quat_z.'];
        S.(name).C_ba = quatToDcm(S.(name).q_ba);
        
        % Extract position measurements
        pos_x = data(:,col_RX(lv1)+4);
        pos_y = data(:,col_RX(lv1)+5);
        pos_z = data(:,col_RX(lv1)+6);
        
        S.(name).r_zw_a = [pos_x.';
                           pos_y.';
                           pos_z.'];
                       
        % Now, we will check if the silly user set the mocap body frame to
        % have a y-axis be up. Fix it for them if they did that. Shame!
        S = mocapSetBodyFrameZaxisUp(S, name);        
    end
else
    error('ViconCsvToStruct requires quaternion outputs from the Vicon system.')
end
    
%% Step 4 - For each ID, extract time range where the object was outside
%           the Mocap coverage area.
% TODO: add visualization for this
% TODO: some gaps not being detected (i.e. gap of length 1, see
% test_dataset9)
objectNames = fieldnames(S);
objectNum   = length(objectNames);
for lv1=1:1:objectNum
    object = S.(objectNames{lv1});
    S.(objectNames{lv1}).gapIntervals = mocapGetGapIntervals(object);
end

%% Step 5 - For each ID, extract time range where the object is stationary.
objectNames = fieldnames(S);
objectNum   = length(objectNames);
stdDevThreshold = 0.001;
windowSize = 1;
for lv1=1:1:objectNum
    object = S.(objectNames{lv1});
    S.(objectNames{lv1}).staticIntervals = ...
        mocapGetStaticIntervals(object, windowSize, stdDevThreshold);
end

end
function y = stringincell(x,str)
% Checks if contents in a cell "x" match the string "str". Using a
% try-catch prevents throwing an error when "x" and "str" are different
% data types.
try
    y = strcmp(x,str);
catch
    y = false;
end
end


