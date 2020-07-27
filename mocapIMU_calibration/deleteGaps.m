function dataCleaned = deleteGaps(data, gaps)
% A function that takes as input synced IMU and Mocap data, as well as
% regions of time which are to be disregarded, and outputs a cleaned
% version of the IMU and Mocap data.

    % Number of gaps
    numGaps = size(gaps, 2);
    
    % Find the indices of the input data that lie within the input gaps
    gapIndices = zeros(length(data.t), 1);
    for lv1=1:1:numGaps
        temp = data.t > gaps(1,lv1) & data.t < gaps(2,lv1);
        gapIndices = gapIndices + temp;
    end
    gapIndices = ~logical(gapIndices);
    
    % Remove those indices from all the data
    dataCleaned = data;
    dataCleaned.t = dataCleaned.t(gapIndices);
    dataCleaned.accIMU = dataCleaned.accIMU(:,gapIndices);
    dataCleaned.omegaIMU = dataCleaned.omegaIMU(:,gapIndices);
    dataCleaned.accMocap = dataCleaned.accMocap(:,gapIndices);
    dataCleaned.omegaMocap = dataCleaned.omegaMocap(:,gapIndices);
end