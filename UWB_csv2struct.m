function S = UWB_csv2struct(filename, t0)
% Extracts UWB data.
% The input t0 synchronizes the UWB data with the IMU.
    
    if nargin < 2
        t0 = 0; % no time synchronization.
    end

    opts = detectImportOptions(filename, 'Delimiter', ',');
    header_row_range = [num2str(1),':',num2str(opts.DataLine(1)+1)];

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
    
    % Timestep
    index = find(contains(headers(headerRow,:),'Timestamp'));
    
    % If the data is organized side by side, reorganize.
    if length(index) > 1
        [data, headers] = horToVer(data, headers, index);
        index = index(1);
    end
    
    S.t = data(:,index);
    % ensure unit is seconds
    if contains(headers(headerRow,index),'(ns)') 
        S.t = S.t./10^9;        % ns to s
    elseif ~contains(headers(headerRow,index),'(s)')
        warning('Unrecognized time unit.')
    end
    S.t = S.t - t0; % Synchronize with the IMU data

    % UWB data
    for lv1=1:1:length(headers)
        tag = cell(2,1);
        if contains(headers(headerRow,lv1),'Range')
            % extract names of UWB tags
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
  
            % save 
            S.(['tags', '_', tag{1}, '_', tag{2}]) = data(:,lv1);
            
            % ensure all units are in meters
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

function [dataNew, headersNew] = horToVer(data, headers, index)
    numData = size(data,1);
    dataNew = data;
    for lv1=2:1:length(index)
        % find key columns associated with the current set of data
        timestepCol = index(lv1);
        firstCol    = index(lv1)+1;
        try 
            lastCol = index(lv1+1)-1;
        catch
            lastCol = size(data,2);
        end
        
        % update the data 
        dataNew = [dataNew(1:end, 1:timestepCol-1), NaN(size(data,1), lastCol-firstCol+1);...
                   data(1:numData,timestepCol), NaN(numData, timestepCol-index(1)-1), data(1:numData, firstCol:lastCol)];
    end
    
    % Update the headers
    headersNew = headers;
    for lv1=length(index):-1:2
        headersNew(:,index(lv1)) = [];
    end
    
    % sort according to the timestamp
    [~,idx] = sort(dataNew(:,index(1)));
    dataNew = dataNew(idx,:);
end

