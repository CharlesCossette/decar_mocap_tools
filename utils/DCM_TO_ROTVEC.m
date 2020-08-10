function output_list = DCM_TO_ROTVEC(C_ba_list)
%%% Converts a DCM C_ba to a rotation vector by calculating the axis and angle.
output_list = zeros(3, size(C_ba_list,3));
for lv1 = 1:size(C_ba_list,3)
    C_ba = C_ba_list(:,:,lv1);
    if C_ba == 0
        % Should never happen.
        output = [0;0;0];
        return
    elseif C_ba == eye(3)
        % No rotation.
        output = [0;0;0];
        return
    elseif issymmetric(C_ba) || (trace(C_ba) > 3)
        % Symmetric, need to use other slower formula.
        output = VEX(-logm(C_ba));
    else
        % Slightly faster algorithm.
        a       = [C_ba(2,3) - C_ba(3,2);
                   C_ba(3,1) - C_ba(1,3);
                   C_ba(1,2) - C_ba(2,1)];
        
        a       = a./norm(a); % Axis
        
        phi     = acos((trace(C_ba) - 1)/2); % Angle
        
        output  =  a*phi;
    end
    output_list(:,lv1) = output;
end
end

