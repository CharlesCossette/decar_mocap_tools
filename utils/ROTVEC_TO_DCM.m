function [output] = ROTVEC_TO_DCM(X)
% Converts a Rotation Vector to a DCM C_ba (the vector is parallel to the axis
% of rotation, and the magnitude is the angle rotated about that axis.)
if length(X) ~= 3
    error('The input must be a vector of length 3')
elseif X == 0
    output  = eye(3);
    return
end


a       = X/norm(X);
phi     = norm(X);

C       = cos(phi)*eye(3) + (1 - cos(phi))*(a*a') - sin(phi)*CrossOperator(a);

output  = C;

end