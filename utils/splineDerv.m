function s = splineDerv(pp,t,q)

    n = pp.order - 1;
    D = diag(n:-1:1,1);
    for lv1 = 1:q
        pp.coefs = pp.coefs*D;
    end
    
    s = ppval(pp,t);
end