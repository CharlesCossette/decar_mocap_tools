function [t_span, x_out] = ode4(odefun, t_span, x0, options)
% ODE4  Classical Runge-Kutta ODE solver with fixed time step.


if ~exist('options','var')
    options = struct();
end

x = x0;
x_out = zeros(length(t_span), length(x0));
x_out(1,:) = x.';
counter = 2;
for lv1 = 1:length(t_span)-1
    t = t_span(lv1);
    dt = t_span(lv1 + 1) - t_span(lv1);
    k1 = odefun(t, x);
    k2 = odefun(t + dt/2, x + dt*k1/2);
    k3 = odefun(t + dt/2, x + dt*k2/2);
    k4 = odefun(t + dt, x + dt*k3);
    x = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
    
    if isfield(options,'indices_to_normalize')
        indx = options.indices_to_normalize;
        x(indx) = x(indx)./sqrt(x(indx).'*x(indx));
    end
    x_out(counter,:) = x.';
    counter = counter + 1;
end
end