function s = splineDerv(pp,t,q)
%SPLINEDERV Computes the derivative of a cubic polynomial spline at given
%time points.
% Inputs:
% --------
% pp: struct
%       This struct is automatically generated by the built in function
%           pp = spline(x,y,xq)
% t: [N x 1] double
%       Time points to look up the spline.
% q: int
%       Derivative number, 1 for 1st derivative, 2 for second
%
% Outputs:
% --------
% s: [n x N] double
%       spline values at t

    n = pp.order - 1;
    D = diag(n:-1:1,1);
    for lv1 = 1:q
        pp.coefs = pp.coefs*D;
    end
    
    s = ppval(pp,t);
end