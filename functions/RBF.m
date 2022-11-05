function k = RBF(X1, X2, sigma, l)
    
    W = diag([l(1),l(2),l(1),l(2),l(1),l(2)]);
    
    if isvector(X1) && isvector(X2)
        % Vector/Vector
        diff = X1 - X2;
        k = sigma .* exp(-0.5 .* (diff' * W * diff));
        
    elseif ~isvector(X1) && ~isvector(X2)
        % Matrix/Matrix
        s1 = size(X1,2);
        s2 = size(X2,2);
        k = zeros(s1,s2);
        for i = 1:s1
            vec = X1(:,i);
            for j = 1:s2
                diff = vec - X2(:,j);
                k(i,j) = sigma .* exp(-0.5 .* (diff' * W * diff));
            end
        end
    elseif ~isvector(X1) && isvector(X2)
        % Matrix/Vector
        s = size(X1);
        k = zeros(s(2),1);
        for i = 1:s(2)
            diff = X1(:,i) - X2;
            k(i,1) = sigma .* exp(-0.5 .* (diff' * W * diff));
        end
    end
    return
    
end