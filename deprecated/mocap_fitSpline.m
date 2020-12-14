function spline_mocap = mocap_fitSpline(data_mocap, gap_size, visual_bool)
% MOCAP_FITSPLINE [DEPRECATED] Fits a spline to all rigid bodies in 'data'.
% Requires the bspline code from decar_utils.

spline_mocap = mocapFitSpline(data_mocap, gap_size, visual_bool);