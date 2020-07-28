function S = UWB_csv2struct(filename, t0)
% Extracts UWB data.
% The input t0 synchronizes the UWB data with the IMU.

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
        if contains(headers(headerRow,lv1),'Range')
            % name of first UWB tag
            tag1 = headers(headerRow-1,lv1);
            tag1 = tag1{1};
            
            % name of second UWB tag
            temp = headers(headerRow,lv1);
            temp = temp{1};
            tagNameIndex = strfind(temp, '0x');
            tag2 = temp(tagNameIndex:tagNameIndex+5);
            
            % save 
            S.(['tags', '_', tag1, '_', tag2]) = data(:,lv1);
            
            % ensure all units are in meters
            if contains(headers(headerRow,lv1),'(mm)') 
                S.(['tags', '_', tag1, '_', tag2]) = S.(['tags', '_', tag1, '_', tag2])/1000;
            elseif contains(headers(headerRow,lv1),'(cm)') 
                S.(['tags', '_', tag1, '_', tag2]) = S.(['tags', '_', tag1, '_', tag2])/100;
            elseif ~contains(headers(headerRow,lv1),'(m)')
                warning('Unrecognized range unit.')
            end    
        end
    end

end

