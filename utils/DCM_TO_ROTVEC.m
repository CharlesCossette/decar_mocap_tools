function output_list = DCM_TO_ROTVEC(C_ba_list)
%%% Converts a DCM C_ba to a rotation vector by calculating the axis and angle.
output_list = zeros(3, size(C_ba_list,3));
for lv1 = 1:size(C_ba_list,3)
    C = C_ba_list(:,:,lv1);
    phi = acos((trace(C) - 1)/2);
    
    
    if abs(phi) < 1e-15 % Then C should be identity.
        phi_cross = logmTaylor(C); % Use taylor series expansion.
    else
        a = -1/(2*sin(phi))*[C(2,3) - C(3,2);
                             C(3,1) - C(1,3);
                             C(1,2) - C(2,1)];
        phi_cross = CrossOperator(a*phi);
    end
    output_list(:,lv1) = VEX(phi_cross);
end

end

function output = VEX(M)
output = [-M(2,3);
    M(1,3);
    -M(1,2)];
end

function X = logmTaylor(A)
% Direct taylor series expansion of the matrix logarithm
X_old = 1000*eye(size(A));
X = zeros(size(A));
k = 1;

if min(real(eig(A))) < 0
   warning('Negative real eigenvalues, using MATLAB built-in logm function.')
   X = logm(A); 
else
    while norm(X - X_old, 'fro') > 1e-15
        normOld = norm(X - X_old, 'fro');
        X_old = X;
        X = X + (-1)^(k+1)/k*(A - eye(size(A)))^k;
        k = k + 1;
        
        normNew = norm(X - X_old, 'fro');
        
        if normNew - normOld> 0
            warning('Series is divergent, using MATLAB built-in logm function.')
            X = logm(A);
        end
    end
    
end
end