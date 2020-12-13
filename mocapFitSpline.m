function spline_mocap = mocapFitSpline(data_mocap, gap_size, visual_bool)
% Fits a spline to all rigid bodies in 'data'.
% Requires the bspline code from decar_utils.

if nargin < 1
    error('Data required');
end
if nargin < 2 || isempty(gap_size)
    gap_size = 1; % set the default downsampling factor for spline
end
if nargin < 3
    visual_bool = false; % visualizations turned off by default
end

% initialize struct to store the spline parameters of each rigid body.
spline_mocap = struct();

% iterate through each identified object
rigid_bodies = fieldnames(data_mocap);
for lv1=1:1:numel(rigid_bodies)
    body_name = rigid_bodies(lv1);
    
    % ensure it is actually a rigid body and not a marker
    if strcmp(data_mocap.(body_name{1}).type, 'Rigid Body')
        
        t = data_mocap.(body_name{1}).t(:);
        waypoints = [data_mocap.(body_name{1}).r_zw_a;
            data_mocap.(body_name{1}).q_ba];
        
        % remove waypoints with missing data
        gap_indices = getIndicesFromIntervals(t,data_mocap.(body_name{1}).gapIntervals);
        t = t(~gap_indices);
        waypoints = waypoints(:, ~gap_indices);
      
        % reduce the number of points to speed up the process of fitting a B-spline.
        t         = t(:,1:gap_size:end);
        waypoints = waypoints(:,1:gap_size:end);

        % Generate the defining properties of the B-spline.
        % Assume initial and final velocity, angular velocity are 0
        %pp = spline(t,waypoints);
        pp = csaps(t,waypoints,0.999999);
        %pp = fn2fm(spaps(t,waypoints,0.00001,[],2),'pp')
        
        % Saving the computed B-spline fit.
        spline_mocap.(body_name{1}) = pp;
        spline_mocap.(body_name{1}).gapIntervals = data_mocap.(body_name{1}).gapIntervals;
        spline_mocap.(body_name{1}).staticIntervals = data_mocap.(body_name{1}).staticIntervals;
        % Evaluating the performance of the fit, visually
        % user-defined trigger
        if visual_bool
            plotScript(data_mocap,t,spline_mocap,body_name)
        end
        
    end
    
end

end

function plotScript(data, t, spline_mocap, body_name)

spline_points = ppval(spline_mocap.(body_name{1}),t);
staticIndices = getIndicesFromIntervals(t, data.(body_name{1}).staticIntervals);
figure
subplot(3,1,1)
plot(data.(body_name{1}).t, data.(body_name{1}).r_zw_a(1,:))
hold on
plot(t,spline_points(1,:))
plot(t,staticIndices*4,'Linewidth',2,'color','black');
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$x$ [m]', 'Interpreter', 'Latex')
legend('Raw Data', 'Bspline fit', 'Static Detector')
title(['Position', body_name])

subplot(3,1,2)
plot(data.(body_name{1}).t, data.(body_name{1}).r_zw_a(2,:))
hold on
plot(t,spline_points(2,:))
plot(t,staticIndices*4,'Linewidth',2,'color','black');
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$y$ [m]', 'Interpreter', 'Latex')

subplot(3,1,3)
plot(data.(body_name{1}).t, data.(body_name{1}).r_zw_a(3,:))
hold on
plot(t,spline_points(3,:))
plot(t,staticIndices*4,'Linewidth',2,'color','black');
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$z$ [m]', 'Interpreter', 'Latex')

figure
subplot(4,1,1)
plot(data.(body_name{1}).t, data.(body_name{1}).q_ba(1,:))
hold on
plot(t,spline_points(4,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$q_1$ [rad]', 'Interpreter', 'Latex')
legend('Raw Data', 'Bspline fit')
title(['Quaternion:', body_name])

subplot(4,1,2)
plot(data.(body_name{1}).t, data.(body_name{1}).q_ba(2,:))
hold on
plot(t,spline_points(5,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$q_2$ [rad]', 'Interpreter', 'Latex')

subplot(4,1,3)
plot(data.(body_name{1}).t, data.(body_name{1}).q_ba(3,:))
hold on
plot(t,spline_points(6,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$q_3$ [rad]', 'Interpreter', 'Latex')

subplot(4,1,4)
plot(data.(body_name{1}).t, data.(body_name{1}).q_ba(4,:))
hold on
plot(t,spline_points(7,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$q_4$ [rad]', 'Interpreter', 'Latex')


figure
plot3(data.(body_name{1}).r_zw_a(1,:),data.(body_name{1}).r_zw_a(2,:),data.(body_name{1}).r_zw_a(3,:))
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