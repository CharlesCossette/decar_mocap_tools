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