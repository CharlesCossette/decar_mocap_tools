function  S = mocap_csv2struct(filename)
%MOCAP_CSV2STRUCT Extracts the relevant information from the raw Optitrack
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
% This transformation is equivalently done below in quaternions.
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

% Use built-in matlab function to automatically detect the header rows.
opts = detectImportOptions(filename);
header_row_range = [num2str(1),':',num2str(opts.DataLine(1)+2)];

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

%% Step 2 - Get unique IDs
is_id = cellfun(@(x) stringincell(x,'ID'),headers);
[row_ID,col_ID] = find(is_id);

IDs = unique(headers(row_ID, col_ID + 1:end));

%% Step 3 - For each ID, get name, position, and attitude data
% The following logical arrays are the same size as the "headers" cell
% array.
% Logical array, true if cell contains "Rigid Body Marker"
is_rgb_marker = cellfun(@(x)stringincell(x,'Rigid Body Marker'),headers);
% Logical array, true if cell contains "Rotation"
is_rot = cellfun(@(x)stringincell(x,'Rotation'),headers);
% Logical array, true if cell contains "Position"
is_pos = cellfun(@(x)stringincell(x,'Position'),headers);
% Logical arrays, true if cell contains "X,Y,Z,W"
is_x = cellfun(@(x)stringincell(x,'X'),headers);
is_y = cellfun(@(x)stringincell(x,'Y'),headers);
is_z = cellfun(@(x)stringincell(x,'Z'),headers);
is_w = cellfun(@(x)stringincell(x,'W'),headers);

for lv1 = 1:numel(IDs)
    % ID of current object
    id = IDs{lv1};
    
    % Logical array, true if cell contains "id"
    is_id = cellfun(@(x)stringincell(x,id),headers);
    % Row and column number of first occurence of "id"
    [row_ID,col_ID] = find(is_id);
    
    % Extract name and type, located in the cells above
    name = headers{row_ID(1) - 1, col_ID(1)};
    name = strrep(name,' ','');
    name = strrep(name,':','');
    type = headers{row_ID(1) - 2, col_ID(1)};
    
    % Column numbers of quaternion and position data.
    % We are looking for the columns that contain a "True" for is_id,
    % is_rot/is_pos, is_x, and a "False" if it is a "Rigid Body Marker"
    % type. This can be compactly evaluated with the following code.
    [~,quat_x_col] = find(sum(is_id + is_rot + is_x - is_rgb_marker,1) == 3);
    [~,quat_y_col] = find(sum(is_id + is_rot + is_y - is_rgb_marker,1) == 3);
    [~,quat_z_col] = find(sum(is_id + is_rot + is_z - is_rgb_marker,1) == 3);
    [~,quat_w_col] = find(sum(is_id + is_rot + is_w - is_rgb_marker,1) == 3);
    [~,pos_x_col] = find(sum(is_id + is_pos + is_x - is_rgb_marker,1) == 3);
    [~,pos_y_col] = find(sum(is_id + is_pos + is_y - is_rgb_marker,1) == 3);
    [~,pos_z_col] = find(sum(is_id + is_pos + is_z - is_rgb_marker,1) == 3);
    
    S.(name).t = data(:,2);
    S.(name).type = type;
    
    if ~isempty(quat_x_col)
        % If a quaternion is available, add it to the data.
        q_ba_mocap = [data(:,quat_w_col).';
            data(:,quat_x_col).';
            data(:,quat_y_col).';
            data(:,quat_z_col).'];
        
        % AXES SWITCHED SO Z POINTS UP
        % Here we apply a frame rotation at the quaternion level. The
        % formulas are taken from (1.49) of 
        % Spacecraft Dynamics and Control: An Introduction
        % de Ruiter, Anton H.J. ;Damaren, Christopher J.; Forbes, James R.
        %
        % The reason we do this at the quaternion level, instead of
        % changing to DCMs, applying the transformation, and changing back
        % to quaternions, is that it introduces discontinuities in the
        % quaternion trajectory due to their ambiguous nature. 
        %
        % Notation: Quaternions are represented as q = [eta; epsilon] where
        % eta is the vector part. 
        etas_old = q_ba_mocap(1,:);
        eps_old = q_ba_mocap(2:4,:);
        eps_switch = 0.5*ones(3,1);
        eta_switch = 0.5;
        etas_new = eta_switch*etas_old - eps_switch.'*eps_old; % Formulas from above.
        eps_new = etas_old.*eps_switch + eta_switch*eps_old + CrossOperator(eps_switch)*eps_old;
        S.(name).q_ba = [etas_new;eps_new];
        
        % REQUIRES AEROSPACE TOOLBOX
        S.(name).C_ba = quat2dcm(S.(name).q_ba.');
    end
    
    % AXES SWITCHED SO Z POINTS UP
    S.(name).r_zw_a = [data(:,pos_z_col).';
                       data(:,pos_x_col).';
                       data(:,pos_y_col).'];
end

%% Step 4 - For each ID, extract time range where the object was outside
%           the Mocap coverage area.

% User-defined parameters
% TODO: 1) decide whether it's worth having these as inputs to the function
thresDiff = 1; % the maximum gap in seconds in which two sets of missing 
               % data are considered to belong to the same time range
bufferSize = 1; % the size of the gap before and after missing data to be 
                % considered as missing data as well. Defined in seconds.

objectNames = fieldnames(S);
objectNum   = length(objectNames);
for lv1=1:1:objectNum
    object = S.(objectNames{lv1});
    t      = object.t';
    
    % Extract the waypoints based on the Mocap readings.
    if strcmp(object.type, 'Rigid Body')
        waypointsIter = [object.r_zw_a; object.q_ba];
    else
        waypointsIter = object.r_zw_a;
    end
    
    % Find timesteps where there is missing data.
    tMissing = t(:, ~any(waypointsIter,1));
    
    rangeIgnore = [];
    if ~isempty(tMissing)
        % Find the time difference between the datapoints with missing data.
        tMissingDiff = [tMissing(1), diff(tMissing)];
        
        % Find the indices of datapoints where the time difference exceeds a
        % threshold.
        tMissingIndices = find(tMissingDiff>thresDiff);
        if ~isempty(tMissingIndices)
            if tMissingIndices(end) ~= length(tMissing)
                tMissingIndices = [tMissingIndices, length(tMissing)];
            end
            for lv2=1:1:length(tMissingIndices)-1
                % check if solo point or within a range of data
                if tMissing(tMissingIndices(lv2)+1) - tMissing(tMissingIndices(lv2)) < thresDiff

                    % compute lower limit
                    lower = tMissing(tMissingIndices(lv2)) - bufferSize;
                    if lower < 0; lower = 0; end

                    % compute upperlimit
                    upper = tMissing(tMissingIndices(lv2+1)-1) + bufferSize;

                    % save range
                    if isempty(rangeIgnore)
                        rangeIgnore = [rangeIgnore, [lower; upper]];
                    elseif lower > rangeIgnore(2,end)
                        rangeIgnore = [rangeIgnore, [lower; upper]];
                    else
                        rangeIgnore(2,end) = upper;
                    end
                end
            end
        end
    end
    
    
    
    S.(objectNames{lv1}).mocapGaps = rangeIgnore;
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

function x = vee(X)
% The "uncross" operator.
x = [-X(2,3);X(1,3);-X(1,2)];
end

function omega_ba_b = quatrate2omega(q_ba, q_ba_dot)
eta = q_ba(1);
epsilon = q_ba(2:4);
S = [-2*epsilon, 2*(eta*eye(3) - CrossOperator(epsilon))];
omega_ba_b = S*q_ba_dot;
end