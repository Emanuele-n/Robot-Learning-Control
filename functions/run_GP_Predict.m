function [mu, var] = run_GP_Predict(sigma_n, sigma, l, x_star)
    
    % - - - - - - - - - - - - - - - - - - -
    % input:
    % sigma_n, sigma, l: GP Optimal Parameters
    % x_star: Test Point
    % - - - - - - - - - - - - - - - - - - -
    % output:
    % mu: Predicted Mean
    % var: Predicted Variance
    % - - - - - - - - - - - - - - - - - - -
    
    global input_active input_training output_training
    
    Ki = RBF(input_active, input_active, sigma, l);
    L = chol(Ki + 1e-6*eye(length(Ki)), "lower");
    V = L\RBF(input_active, input_training, sigma, l);
    M = V*V' + sigma_n*eye(size(V,1));
    Lm = chol(M + 1e-6*eye(length(Ki)), "lower");
    beta_I = Lm\(V * output_training');

    k_I_star = RBF(input_active, x_star, sigma, l);
    k_star_star = RBF(x_star, x_star, sigma, l);
    
    mu = k_I_star' * inv(L)' * inv(Lm)' * beta_I;
    var = diag(k_star_star) - norm(L\k_I_star)^2 + sigma_n * norm(inv(Lm) * inv(L) * k_I_star)^2;
    
end