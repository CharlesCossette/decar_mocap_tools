function [output] = ROTVEC_TO_DCM(C_ba)
% Converts a Rotation Vector to a DCM C_ba (the vector is parallel to the axis
% of rotation, and the magnitude is the angle rotated about that axis.)
if length(C_ba) ~= 3
    error('The input must be a vector of length 3')
elseif C_ba == 0
    output  = eye(3);
    return
end


a       = C_ba/norm(C_ba);
phi     = norm(C_ba);

C       = cos(phi)*eye(3) + (1 - cos(phi))*(a*a') - sin(phi)*CrossOperator(a);

output  = C;

end