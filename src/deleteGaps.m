function [dataCleaned, gapIndices] = deleteGaps(dataSynced, gapIntervals)
% A function that takes as input synced IMU and Mocap data, as well as
% regions of time which are to be disregarded, and outputs a cleaned
% version of the IMU and Mocap data.

    gapIndices = getIndicesFromIntervals(dataSynced.t, gapIntervals);
    
    % Remove those indices from all the data
    dataCleaned = dataSynced;
    dataCleaned.t = dataCleaned.t(~gapIndices);
    dataCleaned.accIMU = dataCleaned.accIMU(:,~gapIndices);
    dataCleaned.omegaIMU = dataCleaned.omegaIMU(:,~gapIndices);
    dataCleaned.accMocap = dataCleaned.accMocap(:,~gapIndices);
    dataCleaned.omegaMocap = dataCleaned.omegaMocap(:,~gapIndices);
end