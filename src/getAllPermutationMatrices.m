function C_list = getAllPermutationMatrices(rightHandedOnly)
    if ~exist('rightHanded','var')
        rightHandedOnly = false;
    end
    
    I = eye(3);
    perm_columns = perms([1 2 3]);
    perm_signs = [1 1 1; perms([-1 -1 1]); -1 -1 -1];
    if ~rightHandedOnly
        perm_signs = [perm_signs; perms([-1 1 1])];
    end
    
    perm_signs = unique(perm_signs,'rows');
    
    C_list = zeros(3,3,size(perm_columns,1)*size(perm_signs,1));
    counter = 1;
    for lv1 = 1:size(perm_columns,1)
        for lv2 = 1:size(perm_signs,1)
            temp = I(:,perm_columns(lv1,:));
            C_list(:,:,counter) = diag(perm_signs(lv2,:))*temp;
            counter = counter + 1;
        end
    end
    
end