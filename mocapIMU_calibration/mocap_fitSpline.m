function bSplineStruct = mocap_fitSpline(data, gapSize, visualBool)
% Fits a spline to all rigid bodies in 'data'.

    if nargin < 1
        error('Data required');
    end
    if nargin < 2
        gapSize = 10; % set the default sampling frequency for spline 
                      % fitting to be 1 sample per 10 recorded.
    end
    if nargin < 3
        visualBool = false; % visualizations turned off by default
    end

    % initialize struct to store the spline parameters of each rigid body.
    bSplineStruct = struct();
    
    % iterate through each identified object
    rigidBodies = fieldnames(data);
    for lv1=1:1:numel(rigidBodies)
        bodyName = rigidBodies(lv1);
        
        % ensure it is actually a rigid body and not a marker
        if isfield(data.(bodyName{1}), 'q_ba')
            
            % generate "waypoints", consisting of a sequence of position
            % vectors r_zw_a and rotation vectors.
            k = length(data.(bodyName{1}).r_zw_a);
            waypoints = zeros(6,k);
            waypoints(1:3,:) = data.(bodyName{1}).r_zw_a;
            for lv2=1:1:k
                C_ba = data.(bodyName{1}).C_ba(:,:,lv2);
                waypoints(4:6,lv2) = DCM_TO_ROTVEC(C_ba);
            end
            
            % remove waypoints with missing data
            t = data.(bodyName{1}).t';
            t(:, ~any(waypoints,1)) = [];
            waypoints(:, ~any(waypoints,1)) = [];
            
            % reduce the number of points to speed up the process of fitting a B-spline.
            t         = t(:,1:gapSize:end);
            waypoints = waypoints(:,1:gapSize:end);
                        
            % Generate the defining properties of the B-spline.
            % Assume initial and final velocity, angular velocity are 0
            [~, knots, P] = bsplineInterp(waypoints,t,zeros(6,1),zeros(6,1));
            
            % Saving the compute B-spline fit.
            bSplineStruct.(bodyName{1}).knots = knots;
            bSplineStruct.(bodyName{1}).P = P;
            
            % Evaluating the performance of the fit, visually
            % user-defined trigger
            if visualBool
                figure
                p=3;
                subplot(3,1,1)
                plot(t, waypoints(1,:))
                hold on
                grid
                xlabel('$t$ [s]', 'Interpreter', 'Latex')
                ylabel('$x$ [m]', 'Interpreter', 'Latex')
                subplot(3,1,2)
                plot(t, waypoints(2,:))
                hold on
                grid
                xlabel('$t$ [s]', 'Interpreter', 'Latex')
                ylabel('$y$ [m]', 'Interpreter', 'Latex')
                subplot(3,1,3)
                plot(t, waypoints(3,:))
                hold on
                grid
                xlabel('$t$ [s]', 'Interpreter', 'Latex')
                ylabel('$z$ [m]', 'Interpreter', 'Latex')
                for t = 1:1:t(end)
                    temp = bspline(t,knots,P,p);
                    subplot(3,1,1)
                    scatter(t,temp(1))
                    subplot(3,1,2)
                    scatter(t,temp(2))
                    subplot(3,1,3)
                    scatter(t,temp(3))
                end
            end
        end
    end
end