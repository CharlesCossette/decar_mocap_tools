function  S = mocap_csv2struct(filename)

opts = detectImportOptions(filename);
header_row_range = [num2str(1),':',num2str(opts.DataLine(1))+2];

headers = readcell(filename,'Range',header_row_range);
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
is_rgb_marker = cellfun(@(x)stringincell(x,'Rigid Body Marker'),headers);
is_rot = cellfun(@(x)stringincell(x,'Rotation'),headers);
is_pos = cellfun(@(x)stringincell(x,'Position'),headers);
is_x = cellfun(@(x)stringincell(x,'X'),headers);
is_y = cellfun(@(x)stringincell(x,'Y'),headers);
is_z = cellfun(@(x)stringincell(x,'Z'),headers);
is_w = cellfun(@(x)stringincell(x,'W'),headers);

for lv1 = 1:numel(IDs)
    id = IDs{lv1};
    is_id = cellfun(@(x)stringincell(x,id),headers);
    [row_ID,col_ID] = find(is_id);
    name = headers{row_ID(1) - 1, col_ID(1)};
    name = strrep(name,' ','');
    name = strrep(name,':','');
    
    [~,quat_x_col] = find(sum(is_id + is_rot + is_x - is_rgb_marker,1) == 3);
    [~,quat_y_col] = find(sum(is_id + is_rot + is_y - is_rgb_marker,1) == 3);
    [~,quat_z_col] = find(sum(is_id + is_rot + is_z - is_rgb_marker,1) == 3);
    [~,quat_w_col] = find(sum(is_id + is_rot + is_w - is_rgb_marker,1) == 3);
    [~,pos_x_col] = find(sum(is_id + is_pos + is_x - is_rgb_marker,1) == 3);
    [~,pos_y_col] = find(sum(is_id + is_pos + is_y - is_rgb_marker,1) == 3);
    [~,pos_z_col] = find(sum(is_id + is_pos + is_z - is_rgb_marker,1) == 3);
    
   
    if ~isempty(quat_x_col)
        % If a quaternion is available, add it to the data.
        q_ba_mocap = [data(:,quat_w_col).';
                      data(:,quat_x_col).';
                      data(:,quat_y_col).';
                      data(:,quat_z_col).'];
                     
        C_ba_mocap = quat2dcm(q_ba_mocap.');
        
        % AXES SWITCHED SO Z POINTS UP
        S.(name).C_ba = [C_ba_mocap(:,3,:),C_ba_mocap(:,1,:),C_ba_mocap(:,2,:)];
        
        S.(name).q_ba = dcm2quat(S.(name).C_ba).';
    end
    
    % AXES SWITCHED SO Z POINTS UP
    S.(name).r_zw_a = [data(:,pos_z_col).';
                       data(:,pos_x_col).';
                       data(:,pos_y_col).'];
    S.(name).t = data(:,2);
end

end
function y = stringincell(x,str)
    try 
        y = strcmp(x,str);
    catch
        y = false;
    end
end