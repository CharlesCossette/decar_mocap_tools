classdef LeastSquaresSolver < handle
    %LEASTSQUARESSOLVER Is a general-purpose least squares solver capable
    % of having a mix vector-space variables, and non-vector-space
    % variables, such as SO(3) or SE(3). The user must specify an "update
    % rule" for each optimization variables. 
    %
    % Example use:
    %
    %    vector_space_update = @(x, dx) x + dx;
    %    so3_update = @(x, dx) x*expm(wedge(dx));
    %    err_func = @(x)my_error_func(x, param1, param2);
    %
    %    ls = LeastSquaresSolver(err_func);
    %    ls.addVariable([1;2;3], vector_space_update, 3);
    %    ls.addVariable(eye(3), so3_update, 3);
    %    [x_opt, cost_opt] = ls.optimize();
    %
    % TODO: I guess i could done this in a function instead of a class, but
    % the argument list would be soo long. Also, this should not be
    % user-facing, only back-end.
    
    properties
        error_function
        variables
        cost
        derivative_method
    end
    methods
        function obj = LeastSquaresSolver(err_func)
            % Constructor
            %
            % PARAMETERS:
            % -----------
            % err_func: function handle
            %       Error function that accepts a cell-array argument x,
            %       containing all the optimization variables, and returns
            %       a vector of errors e such that the cost function 
            %       J = 0.5*e.'*e is minimized. If only one optimization
            %       variable exists, it will be passed directly, and not as
            %       a cell array.
            
            obj.error_function = err_func;
            obj.derivative_method = 'complex_step'
            obj.variables = [];
        end
        function addVariable(obj, x_0, update_func, d)
            % Adds a design variable to the optimization problem.
            %
            % PARAMETERS:
            % -----------
            % x_0: [N x M] double
            %       initial guess of the variable.
            % update_func(x, dx): function handle
            %       function handle accepting two arguments x, dx and returning 
            %       updated value of x, x_new. If left blank, we will
            %       assume additive update such that x_new = x + dx;
            % d: int
            %       dimension or degrees of freedom of variable. For
            %       example d = 3 in the case of SO(3).

            n = numel(obj.variables);
            obj.variables(n + 1).value = x_0;
            obj.variables(n + 1).update_function = update_func;
            obj.variables(n + 1).dimension = d;
        end
        
        function [dx, e, H] = computeStep(obj, x)
            % Computes a least-squares descent step.
            % 
            % PARAMETERS:
            % -----------
            % x: cell array
            %       cell array of design variables
            % 
            % RETURNS:
            % --------
            % dx: [N x 1] double
            %       optimization step to take
            % e: [M x 1] double
            %       error function value at x
            % H: [M x N] double
            %       jacobian of error with respect to x 
            
            e = obj.error_function(x);
               
            
            % Compute jacobian.
            H = []; % TODO, can be preallocated.        
            for lv1 = 1:numel(obj.variables)
                d = obj.variables(lv1).dimension;
                update = obj.variables(lv1).update_function;
                x_iter = x{lv1};
                for lv2 = 1:d
                    switch obj.derivative_method
                        case 'complex_step'
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
                            err_jac = imag(obj.error_function(x_jac))./h;
                        case 'finite_difference'
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
                            err_jac = (obj.error_function(x_jac) - e)./h;
                        otherwise
                            error('Derivative method not supported.')
                    end
                    H = [H, err_jac(:)];
                end
            end
            
            if numel(x) == 1
                x = x{1};
            end
            
            % Compute step (LM? GN?)
            dx = -(H.'*H)\(H.'*e(:));
        end
        
        function updateAll(obj,dx)
            % Parses through the overall step dx and updates each design
            % variable internally.
            %
            % PARAMETERS:
            % -----------
            % dx: [N x 1] double
            %       total step of all variables
            
            for lv1 = 1:numel(obj.variables)
                x_iter = obj.variables(lv1).value;
                d      = obj.variables(lv1).dimension;
                update = obj.variables(lv1).update_function;
                
                obj.variables(lv1).value = update(x_iter, dx(1:d));
                dx = dx(d+1:end); % Chop off remaining portion.
            end
            
        end
        
        function [x_opt, cost_opt] = optimize(obj)
            % Iteratively solves the optimization problem by computing the
            % step, and updating the variables until convergence.
            %
            % RETURNS:
            % --------
            % x_opt: cell array
            %       optimal design variables
            % cost_opt: double
            %       optimal cost
            
            % TODO: if we want to implement Levenberg-Marquart this will
            % take some restructuring.. probably cant have "compute step"
            % as its own function. TBD
            
            dx = 10;
            iter = 5;
            while norm(dx) > 1e-6 && iter < 100
                x = {obj.variables(:).value};
                [dx, e] = obj.computeStep(x);
                obj.updateAll(dx);
                obj.cost = 0.5*e(:).'*e(:);
                disp(obj.cost)
            end
            x_opt = {obj.variables(:).value};
            if numel(x_opt) == 1
                x_opt = x_opt{1};
            end
            cost_opt = obj.cost;
        end
    end
end
            