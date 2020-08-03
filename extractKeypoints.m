function varargout = extractKeypoints(filename, rigidBodies, labels)
% This function can be used in the exact same way as
% mocap_getPointInBodyFrame().
    nOutputs  = nargout;
    varargout = cell(1,nOutputs);    

    assert(length(rigidBodies) == length(labels))
    assert(length(rigidBodies) == nOutputs)
    
    % Extract mocap data
    dataMocap = mocap_csv2struct(filename);

    % Extract the distance from the reference point to the IMU for each
    % rigid body speciried and a corresponding label
    for lv1=1:1:length(rigidBodies)
        r_pz_b   = mocap_getPointInBodyFrame(dataMocap,...
                                           rigidBodies{lv1},...
                                           labels{lv1}); 
        varargout{lv1} = r_pz_b;
    end

end