function  S = viconCsvToStruct(filename)
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


%%
% Rewrite the frame numbers as time-stamps
frequency = headers{1,1};
data(:,1) = data(:,1) ./ frequency;

% Find the first column of each rigid body
is_RX = cellfun(@(x) stringincell(x,'RX'),headers);
[~,col_RX] = find(is_RX);

% Save the name, position, and quaternion data to a struct S
if ~isempty(col_RX)
    for lv1=1:numel(col_RX)
        name = headers{2, col_RX(lv1)};
        name = strrep(name,'Global Angle (Quaternion) ', '');
        name = strrep(name,':', '');
        name = name(1:length(name)/2);
        
        S.(name).t = data(:,1);
        S.(name).type = 'Rigid Body'; % Vicon only records bodies
        
        quat_x = data(:,col_RX(lv1));
        quat_y = data(:,col_RX(lv1)+1);
        quat_z = data(:,col_RX(lv1)+2);
        quat_w = data(:,col_RX(lv1)+3);
        pos_x = data(:,col_RX(lv1)+4);
        pos_y = data(:,col_RX(lv1)+5);
        pos_z = data(:,col_RX(lv1)+6);

        S.(name).q_ba = [quat_w.';
                         quat_x.';
                         quat_y.';
                         quat_z.'];
        S.(name).C_ba = quatToDcm(S.(name).q_ba);
        S.(name).r_zw_a = [pos_x.';
                           pos_y.';
                           pos_z.'];
                       
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


