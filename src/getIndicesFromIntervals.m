function indices = getIndicesFromIntervals(t,intervals)
    t = t(:);
    % Find the indices of the input data that lie within the input gaps
    indices = zeros(length(t), 1);
    for lv1=1:size(intervals,2)
        temp = (t >= intervals(1,lv1)) & (t <= intervals(2,lv1));
        indices = indices(:) + temp(:);
    end
    indices = logical(indices);
end
    