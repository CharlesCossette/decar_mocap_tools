function intervals = mocapGetGapIntervals(object)
% TODO: 1) decide whether it's worth having these as inputs to the function
thresDiff = 1; % the maximum gap in seconds in which two sets of missing
% data are considered to belong to the same time range
bufferSize = 1; % the size of the gap before and after missing data to be
% considered as missing data as well. Defined in seconds.

t      = object.t';

% Extract the waypoints based on the Mocap readings.
if strcmp(object.type, 'Rigid Body')
    waypointsIter = [object.r_zw_a; object.q_ba];
else
    waypointsIter = object.r_zw_a;
end

% Find timesteps where there is missing data.
isMissing = ~any(waypointsIter,1);
intervals = getIntervalsFromIndices(t, isMissing, thresDiff, bufferSize);
end
