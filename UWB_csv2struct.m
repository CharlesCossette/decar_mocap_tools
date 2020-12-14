function S = UWB_csv2struct(filename, t0)
% UWB_csv2struct extracts the range information from the csv file generated
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
    
    
    if nargin < 2
        t0 = 0; % no time synchronization.
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
    headerRow = 1;
    while 1
        if headerRow > 100
            break
        end
        try 
            if ~isempty(find(contains(headers(headerRow,:),'Timestamp')))
                break
            else
                headerRow = headerRow + 1;
            end
        catch
            headerRow = headerRow + 1;
        end
    end
    
    % Find the columns that contain the timestamps.
    index = find(contains(headers(headerRow,:),'Timestamp'));
    
    % If the data is organized side by side, reorganize using horToVer().
    if length(index) > 1
        [data, headers] = horToVer(data, headers, index);
        index = index(1);
    end
    
    % Extract the timestamps.
    S.t = data(:,index);
    
    % Ensure unit is seconds
    if contains(headers(headerRow,index),'(ns)') 
        S.t = S.t./10^9;        % ns to s
    elseif ~contains(headers(headerRow,index),'(s)')
        warning('Unrecognized time unit.')
    end
    
    % Synchronize with the IMU data
    S.t = S.t - t0;

    % Extracting the UWB data
    for lv1=1:1:length(headers)
        tag = cell(2,1);
        if contains(headers(headerRow,lv1),'Range')
            % Extract names of the 2 UWB tags
            headersIter = headers(:,lv1);
            i=1;
            for lv2=1:1:length(headersIter)
                tagIndices = strfind(headersIter{lv2}, '0x');
                for lv3=1:1:length(tagIndices)
                    stringIter = headers(lv2,lv1);
                    stringIter = stringIter{1};
                    tag{i}  = stringIter(tagIndices(lv3):tagIndices(lv3)+5);
                    i = i+1;
                end
                
                if i == 3
                    break
                end
            end
  
            % Save to struct
            S.(['tags', '_', tag{1}, '_', tag{2}]) = data(:,lv1);
            
            % Ensure all units are in meters
            if contains(headers(headerRow,lv1),'(mm)') 
                S.(['tags', '_', tag{1}, '_', tag{2}]) = S.(['tags', '_', tag{1}, '_', tag{2}])/1000;
            elseif contains(headers(headerRow,lv1),'(cm)') 
                S.(['tags', '_', tag{1}, '_', tag{2}]) = S.(['tags', '_', tag{1}, '_', tag{2}])/100;
            elseif ~contains(headers(headerRow,lv1),'(m)')
                warning('Unrecognized range unit.')
            end    
        end
    end

end

function [dataVer, headers] = horToVer(dataHor, headers, index)
    numData = size(dataHor,1);
    dataVer = dataHor;
    for lv1=2:1:length(index)
        % Find key columns associated with the current set of data
        timestepCol = index(lv1);
        firstCol    = index(lv1)+1;
        try 
            lastCol = index(lv1+1)-1;
        catch
            lastCol = size(dataHor,2);
        end
        
        % Update the data 
        savedData = dataVer(1:end, 1:timestepCol-1);
        savedDataNaNCols = NaN(size(dataHor,1), lastCol-firstCol+1);
        newDataTimestamps = dataHor(1:numData,timestepCol);
        newDataNaNCols = NaN(numData, timestepCol-index(1)-1);
        newData = dataHor(1:numData, firstCol:lastCol);
        
        dataVer = [savedData, savedDataNaNCols;...
                   newDataTimestamps, newDataNaNCols, newData];
    end
    
    % Update the headers
    for lv1=length(index):-1:2
        headers(:,index(lv1)) = [];
    end
    
    % Sort according to the timestamp
    [~,idx] = sort(dataVer(:,index(1)));
    dataVer = dataVer(idx,:);
end

