function val = Qval(input_set, input_training, output_training, sigma_n, sigma, l, kernel_fun)

    Ki = kernel_fun(input_set, input_set, sigma, l);
    L = chol(Ki + 1e-6*eye(length(Ki)), "lower");
    V = L\kernel_fun(input_set, input_training, sigma, l);
    M = V*V' + sigma_n*eye(size(V,1));
    Lm = chol(M + 1e-6*eye(length(Ki)), "lower");
    beta = Lm\(V * output_training');
    val = V' * inv(Lm)' * beta;
    
end