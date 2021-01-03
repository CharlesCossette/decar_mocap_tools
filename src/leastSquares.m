function [x_opt, cost_opt] = leastSquares(err_func, variables)
%LEASTSQUARES is a customized least squares solver capable of handling a
% mix of different Euclidean and non-Euclidean variables (such as SO(3),
% SE(3), etc). Uses the LEVENBERG-MARQUART method, see
%
% Madsen, K., Nielsen, H. B., & Tingless, O. (2004).
% Methods for Non-Linear Least Squares Problems.
%
% PARAMETERS:
% -----------
% err_func: function handle
%       error function of the form [e, H] = err_func(x) where x is a cell
%       array of all the variables, e is a column of errors, H is an
%       optional output argument specifying the jacobian of e with respect
%       to x. 
% variables: struct array with fields
%   x_0: [n x m] double 
%       initial guess of variable value
%   update_func: function handle
%       update function of the form x_new = update_func(x, dx)
%   dimension: int
%       dimension or degrees of freedom of the variable. I.e. dx will be a
%       [dimension x 1] column
%   disabled: boolean
%       set to true to still feed variable to error function, but not
%       update it.
%
% RETURNS:
% -------
% x_opt: cell array
%       variables that minimize the cost
% cost_opt: double
%       minimal cost value
%
% TODO: add option for analytical Jacobian
% TODO: user toggle to use complex step.

% Test for conventional use: user supplied a single Euclidean variable
if isa(variables,'double')
    variables(1).x_0 = variables;
end

% Create dummy mandatory fields if missing from user-supplied input.
if ~isfield(variables,'disabled')
    variables(1).disabled = [];
end
if ~isfield(variables,'update_func')
    variables(1).update_func = [];
end
if ~isfield(variables,'dimension')
    variables(1).dimension = [];
end

% Create a new container field called "value" which stores current iterate
% value of the optimization varibles. Load default options for missing
% fields.
for lv1 = 1:numel(variables)
    variables(lv1).value = variables(lv1).x_0;
    
    
    if isempty(variables(lv1).update_func)
        variables(lv1).update_func = @(x,dx) x + dx;
    end
    if isempty(variables(lv1).disabled)
        variables(lv1).disabled = false;
    end
    if isempty(variables(lv1).dimension)
        variables(lv1).dimension = numel(variables(lv1).value);
    end
end

step_tol = 1e-8; % TODO: make these user options.
grad_tol = 1e-8;
cost_tol = 1e-4;
max_iter = 100;

% Display message header
S0 = '                LEVENBERG-MARQUART OPTIMIZATION';
disp(S0);
displayHeader()

% Main Loop
derivative_method = 'finite_difference';
iter = 1;
dx = 10;
dJ = 10;
nu = 2;
[H,e] = computeJacobianFiniteDifference(err_func, variables);
cost = 0.5*(e.')*e;
A = H.'*H;
b = H.'*e;
mu = 1e-9*max(diag(A));
while norm(dx) > step_tol && abs(dJ) > cost_tol && norm(b) > grad_tol && iter < max_iter
    % Compute step
    dx = -(A + mu*eye(size(A)))\b;
    
    % Variable values after taking the step.
    variables_new = updateAll(variables, dx);
    
    % Evaluate new cost function value
    x_new = {variables_new(:).value};
    e_new = err_func(x_new);
    cost_new = 0.5*(e_new.')*e_new;
    dJ = (cost - cost_new)/cost; % Percentage change in cost.
    
    gain_ratio = (cost - cost_new)/(0.5*dx.'*(mu*dx - b));
    if gain_ratio > 0
        % Sufficient reduction, set as new point
        variables = variables_new;
        mu = mu*max(1/3,1 - (2*gain_ratio -1)^3);
        nu = 2;
        [H,e] = computeJacobian(err_func, variables, derivative_method);
        cost = 0.5*(e.')*e;
        A = H.'*H;
        b = H.'*e;
    else
        % Otherwise, reject step, try again with more damping.
        mu = mu*nu;
        nu = 2*nu;
    end
    if mod(iter,30) == 0
        displayHeader()
    end
    disp(['   ',sprintf('%3d',iter), '   |  ',sprintf('%0.3e',cost), '  |  ',sprintf('%0.3e',norm(dx)), '  |  ',sprintf('%0.3e',norm(b)), '  |  ',sprintf('%0.3e',mu)]);
    iter = iter + 1;
    
end
% End main loop

x_opt = {variables(:).value};
cost_opt = cost;
end

function [H, e] = computeJacobian(err_func, variables, derivative_method)
% Call the right derivative method depending on user input.
    switch derivative_method
        case 'complex_step'
            [H, e] = computeJacobianComplexStep(err_func, variables);
        case 'finite_difference'
            [H, e] = computeJacobianFiniteDifference(err_func, variables);
        otherwise
            error('Derivative method not supported.')
    end
end

function [H, e] = computeJacobianComplexStep(err_func, variables)
% Computes the Jacobian of e using the complex step and also returns
% function value. Error function MUST be compatible with complex
% arithmetic. 
%
% PARAMETERS:
% -----------
% err_func: function handle
%       As above.
% variables: struct array
%       As above.
%
% RETURNS:
% --------
% H: [N x m] double
%       Jacobian of e with respect to all the optimization variables.
% e: [N x 1] double
%       Value of error at derivative evaluation point.

    x = {variables(:).value};
    e = err_func(x);

    % Compute jacobian.
    dims = [variables(:).dimension];
    is_disabled = [variables(:).disabled];
    
    H = zeros(size(e,1), sum(dims(~is_disabled)));
    counter = 1;
    for lv1 = 1:numel(variables)
        d = variables(lv1).dimension;
        update = variables(lv1).update_func;
        x_iter = x{lv1};
        for lv2 = 1:d
            h = 1e-18;
            dx_jac = zeros(d,1);
            dx_jac(lv2) = h*1j;

            x_iter_jac = update(x_iter, dx_jac);
            x_jac = x;
            if numel(x_jac) == 1
                x_jac = x_iter_jac;
            else
                x_jac{lv1} = x_iter_jac;
            end
            err_jac = imag(err_func(x_jac))./h;
               
            H(:, counter) = err_jac(:);
            counter = counter + 1;
        end
    end
    if counter ~= size(H,2) + 1
        error('DECAR_MOCAP_TOOLS: Program error!')
    end
end

function [H, e] = computeJacobianFiniteDifference(err_func, variables)
% Computes the Jacobian of e using finite difference and also returns
% function value. This should be used if the error function is not
% compatible with the complex step.
%
% PARAMETERS:
% -----------
% err_func: function handle
%       As above.
% variables: struct array
%       As above.
%
% RETURNS:
% --------
% H: [N x m] double
%       Jacobian of e with respect to all the optimization variables.
% e: [N x 1] double
%       Value of error at derivative evaluation point.

    x = {variables(:).value};
    e = err_func(x);

    % Compute jacobian.
    H = zeros(size(e,1), sum([variables(:).dimension]));
    counter = 1;
    for lv1 = 1:numel(variables)
        d = variables(lv1).dimension;
        update = variables(lv1).update_func;
        x_iter = x{lv1};
        for lv2 = 1:d
            h = 1e-8;
            dx_jac = zeros(d,1);
            dx_jac(lv2) = h;

            x_iter_jac = update(x_iter, dx_jac);
            x_jac = x;
            if numel(x_jac) == 1
                x_jac = x_iter_jac;
            else
                x_jac{lv1} = x_iter_jac;
            end
            err_jac = (err_func(x_jac) - e)./h;
            H(:, counter) = err_jac(:);
            counter = counter + 1;
        end
    end
    if counter ~= size(H,2) + 1
        error('DECAR_MOCAP_TOOLS: Program error!')
    end
end

function variables_new = updateAll(variables, dx)
% Parses through the overall step dx and updates each design
% variable internally.
%
% PARAMETERS:
% -----------
% variables: struct array
% dx: [N x 1] double
%       total step of all variables
    
    variables_new = variables;
    
    for lv1 = 1:numel(variables)
        x_iter = variables(lv1).value;
        d      = variables(lv1).dimension;
        update = variables(lv1).update_func;

        variables_new(lv1).value = update(x_iter, dx(1:d));
        dx = dx(d+1:end); % Chop off remaining portion.
    end

end

function displayHeader()
    % Prints the table header to screen.
    S1 = '   Iter  |    Cost     |  Step Norm  |  Grad Norm  |     mu      ';
    S2 = '---------|-------------|-------------|-------------|-------------';
    disp(S1)
    disp(S2)
end
