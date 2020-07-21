function [output] = DCM_TO_ROTVEC(C)
% %%% Converts a DCM C_ba to a rotation vector by calculating the axis and angle.

if C == 0
    % Should never happen.
    output = [0;0;0];
    return
elseif C == eye(3)
    % No rotation.
    output = [0;0;0];
    return
elseif issymmetric(C) || (trace(C) > 3)
    % Symmetric, need to use other slower formula.
    output = VEX(-logm(C));
else
    % Slightly faster algorithm.
    C       = C'; %This formula is actually for Rotation Matrices
    a       = [C(3,2) - C(2,3);
               C(1,3) - C(3,1);
               C(2,1) - C(1,2)];

    a       = a./norm(a); % Axis

    phi     = acos((trace(C) - 1)/2); % Angle

    output  =  a*phi;

end

end

