function [t_span, x_out] = ode1(odefun, t_span, x0, options)
% ODE4  Classical Euler ODE solver with fixed time step.


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
    x = x + dt*odefun(t, x);
    
    if isfield(options,'indices_to_normalize')
        indx = options.indices_to_normalize;
        x(indx) = x(indx)./norm(x(indx));
    end
    x_out(counter,:) = x.';
    counter = counter + 1;
end
end