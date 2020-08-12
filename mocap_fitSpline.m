function splineMocap = mocap_fitSpline(dataMocap, gapSize, visualBool)
% Fits a spline to all rigid bodies in 'data'.
% Requires the bspline code from decar_utils.

if nargin < 1
    error('Data required');
end
if nargin < 2 || isempty(gapSize)
    gapSize = 1; % set the default sampling frequency for spline
    % fitting to be 1 sample per 10 recorded.
end
if nargin < 3
    visualBool = false; % visualizations turned off by default
end

% initialize struct to store the spline parameters of each rigid body.
splineMocap = struct();

% iterate through each identified object
rigidBodies = fieldnames(dataMocap);
for lv1=1:1:numel(rigidBodies)
    bodyName = rigidBodies(lv1);
    
    % ensure it is actually a rigid body and not a marker
    if strcmp(dataMocap.(bodyName{1}).type, 'Rigid Body')
        
        t = dataMocap.(bodyName{1}).t';
        waypoints = [dataMocap.(bodyName{1}).r_zw_a;
            dataMocap.(bodyName{1}).q_ba];
        
        % remove waypoints with missing data
        t = t(:, any(waypoints,1));
        waypoints = waypoints(:, any(waypoints,1));
      
        % reduce the number of points to speed up the process of fitting a B-spline.
        t         = t(:,1:gapSize:end);
        waypoints = waypoints(:,1:gapSize:end);

        % Generate the defining properties of the B-spline.
        % Assume initial and final velocity, angular velocity are 0
        %pp = spline(t,waypoints);
        pp = csaps(t,waypoints,0.9999995);
        % pp = fn2fm(spaps(t,waypoints,0.001,[],3))
        
        % Saving the computed B-spline fit.
        splineMocap.(bodyName{1}) = pp;
        
        % Evaluating the performance of the fit, visually
        % user-defined trigger
        if visualBool
            plotScript(dataMocap,t,splineMocap,bodyName)
        end
    end
end
end

function plotScript(data,t,splineMocap,bodyName)

spline_points = ppval(splineMocap.(bodyName{1}),t);

figure
subplot(3,1,1)
plot(data.(bodyName{1}).t, data.(bodyName{1}).r_zw_a(1,:))
hold on
plot(t,spline_points(1,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$x$ [m]', 'Interpreter', 'Latex')
legend('Raw Data', 'Bspline fit')
title(['Position', bodyName])

subplot(3,1,2)
plot(data.(bodyName{1}).t, data.(bodyName{1}).r_zw_a(2,:))
hold on
plot(t,spline_points(2,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$y$ [m]', 'Interpreter', 'Latex')

subplot(3,1,3)
plot(data.(bodyName{1}).t, data.(bodyName{1}).r_zw_a(3,:))
hold on
plot(t,spline_points(3,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$z$ [m]', 'Interpreter', 'Latex')

figure
subplot(4,1,1)
plot(data.(bodyName{1}).t, data.(bodyName{1}).q_ba(1,:))
hold on
plot(t,spline_points(4,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$q_1$ [rad]', 'Interpreter', 'Latex')
legend('Raw Data', 'Bspline fit')
title(['Quaternion:', bodyName])

subplot(4,1,2)
plot(data.(bodyName{1}).t, data.(bodyName{1}).q_ba(2,:))
hold on
plot(t,spline_points(5,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$q_2$ [rad]', 'Interpreter', 'Latex')

subplot(4,1,3)
plot(data.(bodyName{1}).t, data.(bodyName{1}).q_ba(3,:))
hold on
plot(t,spline_points(6,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$q_3$ [rad]', 'Interpreter', 'Latex')

subplot(4,1,4)
plot(data.(bodyName{1}).t, data.(bodyName{1}).q_ba(4,:))
hold on
plot(t,spline_points(7,:))
hold off
grid on
xlabel('$t$ [s]', 'Interpreter', 'Latex')
ylabel('$q_4$ [rad]', 'Interpreter', 'Latex')


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