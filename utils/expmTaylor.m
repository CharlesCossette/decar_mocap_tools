function X = expmTaylor(A)
% Direct taylor series expansion of the matrix exponential
X_old = 1000*eye(size(A));
X = zeros(size(A));
k = 0;

while norm(X - X_old, 'fro') > 1e-15
    X_old = X;
    X = X + A^k/factorial(k);
    k = k + 1;

end 
end