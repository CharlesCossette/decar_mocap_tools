function varargout = extractUwbBias(splineMocap, dataUWB, r_pz_b, bodyNames, tagNames)
               
    % specify number of outputs
    nOutputs  = nargout;
    varargout = cell(1,nOutputs);
    
    % extract the position and DCM data from the spline
    splineBodies = fieldnames(splineMocap);
    for lv1=1:1:length(splineBodies)
        fieldIter = splineBodies{lv1};
        knots = splineMocap.(fieldIter).knots;
        P     = splineMocap.(fieldIter).P;
        
        % Extract position data
        temp = bspline(dataUWB.t,knots,P,3);
        splineMocap.(fieldIter).r_zw_a = temp(1:3,:);
        
        % Extract attitude data
        splineMocap.(fieldIter).C_ba = zeros(3,3,length(dataUWB.t));
        for lv2=1:1:length(dataUWB.t)
            tempRot = temp(4:7,lv2);
            splineMocap.(fieldIter).C_ba(:,:,lv2) = quat2dcm(tempRot.');
        end
    end
    
    
    fields = fieldnames(dataUWB);
    for lv1=2:1:length(fields)
        fieldIter = fields{lv1};
        dist = zeros(length(dataUWB.t),1);
        for lv2=1:1:length(dataUWB.t)
            tag1 = fieldIter(6:11);
            tag2 = fieldIter(13:18);

            % Find the index of the tag using the "tagNames" variable,
            % which then allows extracting the rigid body from "bodyNames"
            % and the location of the tag relative to the Mocap reference
            % point from "r_pz_b".
            index1 = find(contains(tagNames, tag1));
            index2 = find(contains(tagNames, tag2));

            r_z1w_a = splineMocap.(bodyNames{index1}).r_zw_a(:,lv2);
            C_b1a   = splineMocap.(bodyNames{index1}).C_ba(:,:,lv2);

            r_z2w_a = splineMocap.(bodyNames{index2}).r_zw_a(:,lv2);
            C_b2a   = splineMocap.(bodyNames{index2}).C_ba(:,:,lv2);

            r_z2z1_a = r_z2w_a - r_z1w_a;

            r_p1z1_b = r_pz_b(:,index1);
            r_p2z2_b = r_pz_b(:,index2);

            r_p1z1_a = C_b1a'*r_p1z1_b;
            r_p2z2_a = C_b2a'*r_p2z2_b;

            r_p2p1_a = r_p2z2_a + r_z2z1_a - r_p1z1_a;
            
            dist(lv2) = norm(r_p2p1_a);
        end
        
        % compute average distance bias
        error = dataUWB.(fieldIter).meas - dist;
        error = rmmissing(error);
        varargout{lv1-1} = mean(error);
        
        figure
        title(fields)
        [meas, measIndex] = rmmissing(dataUWB.(fieldIter).meas);
        plot(dataUWB.t(~measIndex), meas - mean(error))
        hold on
        plot(dataUWB.t, dist)
        grid
        xlabel('$t$ [s]', 'Interpreter', 'Latex')
        ylabel('Distance [m]', 'Interpreter', 'Latex')
    end
end