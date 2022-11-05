function loss = LML(x)

    % Logarithmic Marginal Likelihood
    % - - - - - - - - - - - - - - - - - - -
    % input:
    % x: [sigma_n, sigma, l]
    % - - - - - - - - - - - - - - - - - - -
    % output:
    % loss: Loss Value
    % - - - - - - - - - - - - - - - - - - -

    global input_training output_training input_active

    sigma_n = x(:,1);
    sigma = x(:,2);
    l = x(:,3:4);
    
    Ki = RBF(input_active, input_active, sigma, l);
    L = chol(Ki + 1e-6*eye(length(Ki)), "lower");
    K_I_dot = RBF(input_active, input_training, sigma, l);
    V = L\K_I_dot;
    M = V*V' + sigma_n*eye(size(V,1));
    Lm = chol(M + 1e-6*eye(length(Ki)), "lower");
    beta_I = Lm\(V * output_training');
    
    n = size(input_training,2);
    m = size(input_active,2);
    
    loss = (n-m)*log(sqrt(sigma_n)) ...
        + sum(log(diag(Lm))) ...
        + mean(output_training*output_training' - beta_I'*beta_I,"all") / (2*sigma_n) ...
        + n*log(2*pi)/2 ...
        + trace(RBF(input_training, input_training, sigma, l) - V'*V) / (2*sigma_n);
    
end
