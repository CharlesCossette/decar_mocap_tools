function C_list = getAllPermutationMatrices(rightHandedOnly)
    if ~exist('rightHandedOnly','var')
        rightHandedOnly = false;
    end
    
    I = eye(3);
    perm_columns = perms([1 2 3]);
    perm_signs = [1 1 1; perms([-1 1 1]); perms([-1 -1 1]); -1 -1 -1];

    perm_signs = unique(perm_signs,'rows');
    
    C_list = zeros(3,3,size(perm_columns,1)*size(perm_signs,1));
    counter = 1;
    for lv1 = 1:size(perm_columns,1)
        for lv2 = 1:size(perm_signs,1)
            temp = I(:,perm_columns(lv1,:));
            is_right_handed = (abs(det(diag(perm_signs(lv2,:))*temp) - 1) < 1e-13);
            
            if ~is_right_handed && rightHandedOnly
                % Do nothing
            else
                C_list(:,:,counter) = diag(perm_signs(lv2,:))*temp;
                counter = counter + 1;
            end
        end
    end
    
    C_list = C_list(:,:,1:counter-1);
end