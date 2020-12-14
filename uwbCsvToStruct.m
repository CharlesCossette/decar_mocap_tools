function S = uwbCsvToStruct(filename, t_0)
% UWB_csv2struct extracts the range information from the csv file generated
% by a RPI or a Pozyx device, and stores it into a struct. 

% WARNING: Requires MATLAB 2019a or later with the AEROSPACE TOOLBOX.

% PARAMETERS:
% -----------
% filename: [string]
%       filename of Range raw csv, must be on the matlab path.
% t_0: [double]
%       Time to subtract from all the timestamps.
%       the value used to subtract the timestamps of the IMU data collected
%       on the same device as the range measurements. This allows
%       synchronization between the UWB data and the IMU data. 
%
% RETURNS:
% --------
%   S: [struct] with fields
%       t: [N x 1] double.
%           Time points of all the data.
%       tags_0x####_0x####: [N x 1] double
%           Range measurements between the tags specified in the field
%           name.
    
    
    if nargin < 2
        t_0 = 0; % no time synchronization.
    end

    % Use built-in matlab function to automatically detect the header rows.
    % It is okay to take more rows than the headers, but it is not okay to
    % take less rows, so a conservative approach is used.
    opts = detectImportOptions(filename, 'Delimiter', ',');
    header_row_range = [num2str(1),':',num2str(opts.DataLine(1)+1)];

    % Store headers into cell array, data into matrix
    headers = readcell(filename, 'Range',header_row_range, 'Delimiter', ',');
    data = readmatrix(filename, 'NumHeaderLines',opts.DataLine(1), 'Delimiter', ',');

    %% STEP 1 - Remove any columns with barely any data
    % Criteria: If less than 99% of rows have data, delete column.
    to_delete = false(1,size(data,2));
    for lv1 = 1:size(data,2)
        data_col = data(:,lv1);
        indx = logical(isnan(data_col) + (data_col == 0));
        if (sum(indx)/size(data,1)) > 0.99
            to_delete(lv1) = true;
        end
    end

    headers = headers(:,~to_delete);
    data = data(:,~to_delete);

    %% Step 2 - Get UWB data
    % Find the row where the main headers are present
    header_row = 1;
    while 1
        if header_row > 100
            break
        end
        try 
            if ~isempty(find(contains(headers(header_row,:),'Timestamp')))
                break
            else
                header_row = header_row + 1;
            end
        catch
            header_row = header_row + 1;
        end
    end
    
    % Find the columns that contain the timestamps.
    index = find(contains(headers(header_row,:),'Timestamp'));
    
    % If the data is organized side by side, reorganize using horToVer().
    if length(index) > 1
        [data, headers] = horToVer(data, headers, index);
        index = index(1);
    end
    
    % Extract the timestamps.
    S.t = data(:,index);
    
    % Ensure unit is seconds
    if contains(headers(header_row,index),'(ns)') 
        S.t = S.t./10^9;        % ns to s
    elseif ~contains(headers(header_row,index),'(s)')
        warning('Unrecognized time unit.')
    end
    
    % Synchronize with the IMU data
    S.t = S.t - t_0;

    % Extracting the UWB data
    for lv1=1:1:length(headers)
        tag = cell(2,1);
        if contains(headers(header_row,lv1),'Range')
            % Extract names of the 2 UWB tags
            headers_iter = headers(:,lv1);
            i=1;
            for lv2=1:1:length(headers_iter)
                tag_indices = strfind(headers_iter{lv2}, '0x');
                for lv3=1:1:length(tag_indices)
                    string_iter = headers(lv2,lv1);
                    string_iter = string_iter{1};
                    tag{i}  = string_iter(tag_indices(lv3):tag_indices(lv3)+5);
                    i = i+1;
                end
                
                if i == 3
                    break
                end
            end
  
            % Save to struct
            S.(['tags', '_', tag{1}, '_', tag{2}]) = data(:,lv1);
            
            % Ensure all units are in meters
            if contains(headers(header_row,lv1),'(mm)') 
                S.(['tags', '_', tag{1}, '_', tag{2}]) = S.(['tags', '_', tag{1}, '_', tag{2}])/1000;
            elseif contains(headers(header_row,lv1),'(cm)') 
                S.(['tags', '_', tag{1}, '_', tag{2}]) = S.(['tags', '_', tag{1}, '_', tag{2}])/100;
            elseif ~contains(headers(header_row,lv1),'(m)')
                warning('Unrecognized range unit.')
            end    
        end
    end

end

function [data_ver, headers] = horToVer(data_hor, headers, index)
    num_data = size(data_hor,1);
    data_ver = data_hor;
    for lv1=2:1:length(index)
        % Find key columns associated with the current set of data
        timestepCol = index(lv1);
        firstCol    = index(lv1)+1;
        try 
            lastCol = index(lv1+1)-1;
        catch
            lastCol = size(data_hor,2);
        end
        
        % Update the data 
        saved_data = data_ver(1:end, 1:timestepCol-1);
        saved_data_nan_cols = NaN(size(data_hor,1), lastCol-firstCol+1);
        new_data_timestamps = data_hor(1:num_data,timestepCol);
        new_data_nan_cols = NaN(num_data, timestepCol-index(1)-1);
        new_data = data_hor(1:num_data, firstCol:lastCol);
        
        data_ver = [saved_data, saved_data_nan_cols;...
                   new_data_timestamps, new_data_nan_cols, new_data];
    end
    
    % Update the headers
    for lv1=length(index):-1:2
        headers(:,index(lv1)) = [];
    end
    
    % Sort according to the timestamp
    [~,idx] = sort(data_ver(:,index(1)));
    data_ver = data_ver(idx,:);
end

