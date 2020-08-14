function intervals = getIntervalsFromIndices(t,indices,thresDiff,bufferSize)

tMissing = t(:, indices);
intervals = [];
if ~isempty(tMissing)
    % Find the time difference between the datapoints with missing data.
    tMissingDiff = [tMissing(1), diff(tMissing)];
    
    % Find the indices of datapoints where the time difference exceeds a
    % threshold.
    tMissingIndices = find(tMissingDiff>thresDiff);
    if ~isempty(tMissingIndices)
        if tMissingIndices(end) ~= length(tMissing)
            tMissingIndices = [tMissingIndices, length(tMissing)];
        end
        for lv2=1:1:length(tMissingIndices)-1
            % check if solo point or within a range of data
            if tMissing(tMissingIndices(lv2)+1) - tMissing(tMissingIndices(lv2)) < thresDiff
                
                % compute lower limit
                lower = tMissing(tMissingIndices(lv2)) - bufferSize;
                if lower < 0; lower = 0; end
                
                % compute upperlimit
                upper = tMissing(tMissingIndices(lv2+1)-1) + bufferSize;
                
                % save range
                if isempty(intervals)
                    intervals = [intervals, [lower; upper]];
                elseif lower > intervals(2,end)
                    intervals = [intervals, [lower; upper]];
                else
                    intervals(2,end) = upper;
                end
            end
        end
    end
end
end