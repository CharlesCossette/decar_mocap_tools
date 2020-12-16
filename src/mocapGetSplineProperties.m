function spline_properties = mocapGetSplineProperties(data_mocap, gap_size)
% Extracts the defining properties of the B-spline.

    t = data_mocap.t(:);
    waypoints = [data_mocap.r_zw_a;
                 data_mocap.q_ba];

    % remove waypoints with missing data
    gap_indices = getIndicesFromIntervals(t,data_mocap.gapIntervals);
    t = t(~gap_indices);
    waypoints = waypoints(:, ~gap_indices);

    % reduce the number of points to speed up the process of fitting a B-spline.
    t         = t(:,1:gap_size:end);
    waypoints = waypoints(:,1:gap_size:end);

    % Generate the defining properties of the B-spline.
    % Assume initial and final velocity, angular velocity are 0
    %spline_properties = spline(t,waypoints);
    spline_properties = csaps(t,waypoints,0.999999);
    %spline_properties = fn2fm(spaps(t,waypoints,0.00001,[],2),'pp')

end