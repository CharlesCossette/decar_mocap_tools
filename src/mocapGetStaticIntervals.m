function intervals = mocapGetStaticIntervals(object, windowSize, stdDevThreshold)

t      = object.t';
freq = 1/((t(end) - t(1))/numel(t));
windowIndices = round((windowSize/2)*freq);
covThreshold = stdDevThreshold^2;

isStatic = false(size(t));

%Sliding window, centered at time step lv1
for lv1 = windowIndices:(length(t) - windowIndices)
    
    % Covariance of positions inside the window.
    positions = object.r_zw_a(:,lv1 - windowIndices +1: lv1 + windowIndices).';
    positions = positions(~any(isnan(positions),2),:);
    pos_cov = cov(positions);
    
    if norm(diag(pos_cov)) < covThreshold
        % Then it is static
        isStatic(lv1) = true;
        if lv1 == windowIndices
            isStatic(1:windowIndices) = true;
        elseif lv1 == (length(t) - windowIndices)
            isStatic(lv1:end) = true;
        end
    end
end

intervals = getIntervalsFromIndices(t,isStatic,0.4,0);
end