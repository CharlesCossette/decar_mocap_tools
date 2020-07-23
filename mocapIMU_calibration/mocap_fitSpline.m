function bSplineStruct = mocap_fitSpline(data, gapSize, visualBool)
% Fits a spline to all rigid bodies in 'data'.
% Requires the bspline code from decar_utils.

    if nargin < 1
        error('Data required');
    end
    if nargin < 2 || isempty(gapSize)
        gapSize = 5; % set the default sampling frequency for spline 
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
            [knots, P, ~] = bsplineInterp(waypoints,t,zeros(6,1),zeros(6,1));
            
            % Saving the computed B-spline fit.
            bSplineStruct.(bodyName{1}).knots = knots;
            bSplineStruct.(bodyName{1}).P = P;
            
            % Evaluating the performance of the fit, visually
            % user-defined trigger
            if visualBool
                figure
                p=3;
                spline_points = bspline(t,knots,P,p);
                
                subplot(3,1,1)
                plot(data.(bodyName{1}).t, data.(bodyName{1}).r_zw_a(1,:))
                hold on
                plot(t,spline_points(1,:))
                hold off
                grid on
                xlabel('$t$ [s]', 'Interpreter', 'Latex')
                ylabel('$x$ [m]', 'Interpreter', 'Latex')
                legend('Raw Data', 'Bspline fit')
                title('Position')
                
                subplot(3,1,2)
                plot(t, waypoints(2,:))
                hold on
                plot(t,spline_points(2,:))
                hold off
                grid on
                xlabel('$t$ [s]', 'Interpreter', 'Latex')
                ylabel('$x$ [m]', 'Interpreter', 'Latex')
                
                subplot(3,1,3)
                plot(t, waypoints(3,:))
                hold on
                plot(t,spline_points(3,:))
                hold off
                grid on
                xlabel('$t$ [s]', 'Interpreter', 'Latex')
                ylabel('$z$ [m]', 'Interpreter', 'Latex')
                
                figure
                plot3(data.(bodyName{1}).r_zw_a(1,:),data.(bodyName{1}).r_zw_a(2,:),data.(bodyName{1}).r_zw_a(3,:))
                hold on
                plot3(spline_points(1,:),spline_points(2,:),spline_points(3,:))
                hold off
                grid on
                axis vis3d
                xlabel('$x$ [m]', 'Interpreter', 'Latex')
                ylabel('$y$ [m]', 'Interpreter', 'Latex')
                zlabel('$z$ [m]', 'Interpreter', 'Latex')
                legend('Raw Data','Bspline fit')
            end
        end
    end
end