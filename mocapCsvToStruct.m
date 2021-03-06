function  S = mocapCsvToStruct(filename)
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

% Use built-in matlab function to automatically detect the header rows.
opts = detectImportOptions(filename);
header_row_range = [num2str(1),':',num2str(7)];

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
        % Notation: Quaternions are represented as q = [epsilon; eta] where
        % eta is the vector part.
        q_aaprime = 0.5*ones(4,1); % Quaternion corresponding to C_aa'
        S.(name).q_ba =  quatMult(q_ba_mocap, q_aaprime);
        
        S.(name).C_ba = quatToDcm(S.(name).q_ba);
        
        % Now, we will check if the silly user set the mocap body frame to
        % have a y-axis be up. Fix it for them if they did that. Shame!
        r_up_a = [0;0;1];
        r_up_b = zeros(3,50);
        for lv2 = 1:size(r_up_b,2)
            r_up_b(:,lv2) = S.(name).C_ba(:,:,lv2)*r_up_a;
        end
        r_up_b = mean(r_up_b,2,'omitnan');
        if norm(r_up_a - r_up_b) > 0.2
            % Then the Z-axis is not up! Assuming y axis is up.
            disp('WARNING: We have detected that the Z axis of the body frame of');
            disp([newline, '>>>>>>>> ', name, ' <<<<<<<<', newline]);
            disp('did not start pointing upwards. We cannot tell if this is');
            disp('because the body frame was not defined with Z pointing up, or ')
            disp('if the body is simply not oriented with Z pointing up.');
            disp(newline)
            is_vertical = input(['Was ', name,' vertical (i.e. intended Z-up) ',...
                                newline, ' at the start of the take? (y/n)'],'s');
            switch is_vertical
                case {'y','yes'}
                
                    disp(['This correction will be made automatically, assuming',...
                        ' that the Y-axis was in fact the up/vertical one.']);
                    q_bprimeb = [-0.5;0.5;0.5;0.5];
                    S.(name).q_ba = quatMult(q_bprimeb,S.(name).q_ba);
                    S.(name).C_ba = quatToDcm(S.(name).q_ba);
                    S.(name).q_ba = dcmToQuat(S.(name).C_ba);

                    % Repeat the test again, make sure the problem is fixed.
                    for lv2 = 1:size(r_up_b,2)
                        r_up_b(:,lv2) = S.(name).C_ba(:,:,lv2)*r_up_a;
                    end
                    r_up_b = mean(r_up_b,2,'omitnan');
                    if norm(r_up_a - r_up_b) > 0.2
                        error('Program error!') 
                    end
                otherwise
                    disp('The body frame is left unmodified.')
            end
            
        end
        
    end
    
    % AXES SWITCHED SO Z POINTS UP
    C_aaprime = [0 1 0; 0 0 1; 1 0 0];
    S.(name).r_zw_a = C_aaprime.'*[data(:,pos_x_col).';
                                   data(:,pos_y_col).';
                                   data(:,pos_z_col).'];
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


