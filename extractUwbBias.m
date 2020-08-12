function dataUwbCorrected = extractUwbBias(splineMocap, dataUWB, r_pz_b, bodyNames, tagNames)
    
    % initialize struct to store new corrected data
    dataUwbCorrected = struct();

    % extract the position and DCM data from the spline
    splineBodies = fieldnames(splineMocap);
    for lv1=1:1:length(splineBodies)
        fieldIter = splineBodies{lv1};
        
        % Extract position data
        temp = ppval(splineMocap.(fieldIter), dataUWB.t);
        splineMocap.(fieldIter).r_zw_a = temp(1:3,:);
        
        % Extract attitude data
        splineMocap.(fieldIter).C_ba = zeros(3,3,length(dataUWB.t));
        for lv2=1:1:length(dataUWB.t)
            tempRot = temp(4:7,lv2);
            splineMocap.(fieldIter).C_ba(:,:,lv2) = quat2dcm(tempRot.');
        end
    end
    
    % iterate through different tag combinations
    fields = fieldnames(dataUWB);
    for lv1=2:1:length(fields)
        fieldIter = fields{lv1};
        dist = zeros(length(dataUWB.t),1);
        
        % find the true distance between the two tags based on Mocap data
        for lv2=1:1:length(dataUWB.t)
            % Extract tag names.
            tag1 = fieldIter(6:11);
            tag2 = fieldIter(13:18);

            % Find the index of the tag using the "tagNames" variable,
            % which then allows extracting the rigid body from "bodyNames"
            % and the location of the tag relative to the Mocap reference
            % point from "r_pz_b".
            index1 = find(contains(tagNames, tag1));
            index2 = find(contains(tagNames, tag2));

            % extract Mocap data
            r_z1w_a = splineMocap.(bodyNames{index1}).r_zw_a(:,lv2);
            C_b1a   = splineMocap.(bodyNames{index1}).C_ba(:,:,lv2);
            r_z2w_a = splineMocap.(bodyNames{index2}).r_zw_a(:,lv2);
            C_b2a   = splineMocap.(bodyNames{index2}).C_ba(:,:,lv2);

            % find the vector between the two reference points.
            r_z2z1_a = r_z2w_a - r_z1w_a;

            % extract the vector between the tag and the reference point
            % in the body frame.
            r_p1z1_b = r_pz_b(:,index1);
            r_p2z2_b = r_pz_b(:,index2);

            % find the vector between the tag and the reference point in
            % the absolute frame.
            r_p1z1_a = C_b1a'*r_p1z1_b;
            r_p2z2_a = C_b2a'*r_p2z2_b;

            % find the vector between the two tags.
            r_p2p1_a = r_p2z2_a + r_z2z1_a - r_p1z1_a;
            
            % find the distance between the two tags.
            dist(lv2) = norm(r_p2p1_a);
        end
        
        % compute error = measured - true, and remove missing values.
        error = dataUWB.(fieldIter) - dist;
        [error_noMissing, error_missingIndex] = rmmissing(error);
        t = dataUWB.t(~error_missingIndex);
        
        % remove outliers so not to affect spline fit.
        [error_noOutliers, outlierIndex] = rmoutliers(error_noMissing);
        
        % fit a spline to the error and extract the values
        vals = csaps(t(~outlierIndex), error_noOutliers, 1E-3, t);
        
        % Extract clean measurements
        [meas_noMissing, meas_missingIndex] = rmmissing(dataUWB.(fieldIter));
        
        % compute cleaned and corrected data
        dataUwbCorrected.(fieldIter).t    = dataUWB.t(~meas_missingIndex);
        dataUwbCorrected.(fieldIter).meas = meas_noMissing - vals;
        
        % Plotting - spline fit
        figure 
        plot(t,error_noMissing)
        hold on
        plot(t,vals)
        grid
        xlabel('$t$ [s]', 'Interpreter', 'Latex')
        ylabel('$e$ [m]', 'Interpreter', 'Latex')
        legend('Error between UWB measurement and ground truth distance', 'Spline fit')
        title(['Bias calibrated distance measurements between tags ', tag1, ' and ', tag2])
        
        % Plotting - final results
        figure
        plot(dataUwbCorrected.(fieldIter).t, dataUwbCorrected.(fieldIter).meas)
        hold on
        plot(dataUWB.t, dist)
        grid
        xlabel('$t$ [s]', 'Interpreter', 'Latex')
        ylabel('$d$ [m]', 'Interpreter', 'Latex')
        legend('UWB Range Measurement', 'Mocap Data')
        title(['Bias calibrated distance measurements between tags ', tag1, ' and ', tag2])
    end
end