function data_uwb_corrected = calibrateUWB(spline_mocap, data_uwb, r_pz_b, body_names, tag_names, remove_outliers)
% calibrateUWB corrects the UWB measurements by removing biases using 
% ground truth data. To do so, a heavily smoothed spline is fit to the
% error between the range measurements and the ground truth distance, which
% is then subtracted from the range measurements.

% Inputs:
% --------
% spline_mocap: [struct]
%       Contains a bunch of spline parameters of the different rigid bodies
%       detected by the Mocap system. Used to extract the ground truth
%       distance between the tags.
% data_uwb: [struct]
%       Contains the range measurements between each pair of tags.
% r_pz_b: [3 x M] double
%       An array where each column represents the position of a tag
%       relative to the reference point of the Mocap system.
% body_names: [1 x M] cell
%       Contains the RigidBody names where each tag lies, in the same tag
%       order specified by r_pz_b. The names come from the convention used
%       by the Mocap system.
% tag_names: [1 x M] cell
%       Contains the serial code of each tag, in the same order specified
%       by r_pz_b. 
% remove_outliers: boolean
%       A boolean input to specify whether or not the output data should include
%       outliers.

% Outputs:
% --------
%   data_uwb_corrected: [struct] with fields
%       tags_0x####_0x####: [struct] with fields
%           t: [N x 1] double.
%              Time points in which measurements were collected between
%              these two tags.
%           meas: [N x 1] double.
%              The measurements between the two tags.
%
% ------------------------------------------------------------------------------

    % initialize struct to store new corrected data
    data_uwb_corrected = struct();

    % extract the position and DCM data from the spline
    spline_bodies = fieldnames(spline_mocap);
    for lv1=1:1:length(spline_bodies)
        field_iter = spline_bodies{lv1};
        
        % Extract position data
        temp = ppval(spline_mocap.(field_iter), data_uwb.t);
        spline_mocap.(field_iter).r_zw_a = temp(1:3,:);
        
        % Extract attitude data
        spline_mocap.(field_iter).C_ba = zeros(3,3,length(data_uwb.t));
        for lv2=1:1:length(data_uwb.t)
            rot_temp = temp(4:7,lv2);
            spline_mocap.(field_iter).C_ba(:,:,lv2) = quat2dcm(rot_temp.');
        end
    end
    
    % iterate through different tag combinations
    fields = fieldnames(data_uwb);
    for lv1=2:1:length(fields)
        field_iter = fields{lv1};
        
        % Extract tag names.
        tag1 = field_iter(6:11);
        tag2 = field_iter(13:18);

        % Find the idx of the tag using the "tag_names" variable,
        % which then allows extracting the rigid body from "body_names"
        % and the location of the tag relative to the Mocap reference
        % point from "r_pz_b".
        idx1 = find(contains(tag_names, tag1));
        idx2 = find(contains(tag_names, tag2));

        % remove missing values
        [meas_rm_missing, meas_missing_idx] = rmmissing(data_uwb.(field_iter));
        t_rm_missing = data_uwb.t(~meas_missing_idx);
        r_z1w_a = spline_mocap.(body_names{idx1}).r_zw_a(:,~meas_missing_idx);
        C_b1a = spline_mocap.(body_names{idx1}).C_ba(:,:,~meas_missing_idx);
        r_z2w_a = spline_mocap.(body_names{idx2}).r_zw_a(:,~meas_missing_idx);
        C_b2a = spline_mocap.(body_names{idx2}).C_ba(:,:,~meas_missing_idx);
        
        dist_rm_missing = zeros(length(t_rm_missing),1);
        
        % find the true distance between the two tags based on Mocap data
        for lv2=1:1:length(t_rm_missing)
            % extract Mocap data
            r_z1w_a_iter = r_z1w_a(:,lv2);
            C_b1a_iter   = C_b1a(:,:,lv2);
            r_z2w_a_iter = r_z2w_a(:,lv2);
            C_b2a_iter   = C_b2a(:,:,lv2);

            % find the vector between the two reference points.
            r_z2z1_a = r_z2w_a_iter - r_z1w_a_iter;

            % extract the vector between the tag and the reference point
            % in the body frame.
            r_p1z1_b = r_pz_b(:,idx1);
            r_p2z2_b = r_pz_b(:,idx2);

            % find the vector between the tag and the reference point in
            % the absolute frame.
            r_p1z1_a = C_b1a_iter'*r_p1z1_b;
            r_p2z2_a = C_b2a_iter'*r_p2z2_b;

            % find the vector between the two tags.
            r_p2p1_a = r_p2z2_a + r_z2z1_a - r_p1z1_a;
            
            % find the distance between the two tags.
            dist_rm_missing(lv2) = norm(r_p2p1_a);
        end
        
        % compute error = measured - true, and remove missing values.
        error_rm_missing = meas_rm_missing - dist_rm_missing;
        
        % extract data without outliers. Will be need so not to affect spline
        % fit.
        [error_rm_outliers, error_outliers_idx] = rmoutliers(error_rm_missing);
        meas_rm_outliers = meas_rm_missing(~error_outliers_idx);
        t_rm_outliers = t_rm_missing(~error_outliers_idx);
        dist_rm_outliers = dist_rm_missing(~error_outliers_idx);
        
        % fit a spline to the error without outliers, and extract the values at
        % the timesteps dependent on whether or not we want outliers in our
        % data.
        if remove_outliers
            t_final = t_rm_outliers;
            vals = csaps(t_rm_outliers, error_rm_outliers, 1E-3, t_final);
            meas_final = meas_rm_outliers;
            dist_final = dist_rm_outliers;
        else
            t_final = t_rm_missing;
            vals = csaps(t_rm_outliers, error_rm_outliers, 1E-3, t_final);
            meas_final = meas_rm_missing;
            dist_final = dist_rm_missing;
        end
        
        % compute cleaned and corrected data
        data_uwb_corrected.(field_iter).t    = t_final;
        data_uwb_corrected.(field_iter).meas = meas_final - vals;
        
%         [data_uwb_corrected.(field_iter).meas, meas_outliers_idx] = rmoutliers(data_uwb_corrected.(field_iter).meas,...
%                                                                           'percentiles', [0, 99]);
%         data_uwb_corrected.(field_iter).t = data_uwb_corrected.(field_iter).t(~meas_outliers_idx);
        
        % ------------------------------ OUTPUTS ------------------------------ %
        
        % Plotting - spline fit (with the outliers)
        figure
        plot(t_rm_missing,error_rm_missing)
        hold on
        plot(t_final,vals)
        grid
        xlabel('$t$ [s]', 'Interpreter', 'Latex')
        ylabel('$e$ [m]', 'Interpreter', 'Latex')
        legend('Error between UWB measurement and ground truth distance', 'Spline fit')
        title(['Spline fit on the error of the distance measurements between tags ', tag1, ' and ', tag2])
        
        % Plotting - final results
        figure
        plot(data_uwb_corrected.(field_iter).t, data_uwb_corrected.(field_iter).meas)
        hold on
        plot(t_rm_missing, dist_rm_missing)
        grid
        xlabel('$t$ [s]', 'Interpreter', 'Latex')
        ylabel('$d$ [m]', 'Interpreter', 'Latex')
        legend('Corrected UWB Range Measurement', 'Mocap Data')
        title(['Bias calibrated distance measurements between tags ', tag1, ' and ', tag2])
        
        % Plotting - final error
        figure
        plot(data_uwb_corrected.(field_iter).t, dist_final - data_uwb_corrected.(field_iter).meas)
        grid
        xlabel('$t$ [s]', 'Interpreter', 'Latex')
        ylabel('$d$ [m]', 'Interpreter', 'Latex')
        title(['Error of bias calibrated distance measurements between tags ', tag1, ' and ', tag2])
        
        % Mean error (should preferably be close to 0)
        ['Mean error between calibrated range measurements of tags ', tag1, ' and ', tag2, ': ',...
         num2str(mean(dist_final - data_uwb_corrected.(field_iter).meas))]
    end
end