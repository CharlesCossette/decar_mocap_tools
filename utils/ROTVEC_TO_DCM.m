function [output] = ROTVEC_TO_DCM(phi_vec)
% Converts a Rotation Vector to a DCM C_ba (the vector is parallel to the axis
% of rotation, and the magnitude is the angle rotated about that axis.)
if length(phi_vec) ~= 3
    error('The input must be a vector of length 3')
elseif all(phi_vec == 0)
    output  = eye(3);
    return
end

a       = phi_vec/norm(phi_vec);
phi     = norm(phi_vec);

C       = cos(phi)*eye(3) + (1 - cos(phi))*(a*a.') - sin(phi)*CrossOperator(a);

output  = C;

end