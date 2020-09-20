function X = crossOp(v)
    % compute the cross operator X of a vector v
    X = [0, -v(3), v(2);...
         v(3), 0, -v(1);...
         -v(2), v(1), 0];
end