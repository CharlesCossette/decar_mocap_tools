function C_ba_list = rotvecToDcm(phi_vec_list)
% Converts a Rotation Vector to a DCM C_ba (the vector is parallel to the axis
% of rotation, and the magnitude is the angle rotated about that axis.)

C_ba_list = zeros(3,3,size(phi_vec_list,2));
for lv1 = 1:size(phi_vec_list,2)
    phi_vec = phi_vec_list(:,lv1);
    if size(phi_vec, 1) ~= 3
        error('The input must be a vector of length 3')
    elseif all(phi_vec == 0)
        C  = eye(3);
    else
        a       = phi_vec/norm(phi_vec);
        phi     = norm(phi_vec);

        C       = cos(phi)*eye(3) + (1 - cos(phi))*(a*a.') - sin(phi)*crossOp(a);
    end
    C_ba_list(:,:,lv1)  = C;
end

end