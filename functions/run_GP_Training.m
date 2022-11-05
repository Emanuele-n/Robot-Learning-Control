function [sigma_n, sigma, l, I] = run_GP_Training(training_set, I, sigma_n, sigma, l, active_p, n, offline_flag)
    

%     global input_active output_active
%     I = datasample(1:size(training_set,2), active_p, "Replace",false)';
%     input_active = training_set(1:6, I);
%     output_active = training_set(7:8, I);

    I = run_GP_Heuristic(training_set, I, active_p, sigma_n, sigma, l, offline_flag);
      
    start_point = [sigma_n,sigma,l];
    [res,~] = fmincon(@LML, start_point, [],[],[],[],zeros(1,n+2),[],[]);

    curr_best_params = res;
    sigma_n = curr_best_params(1);
    sigma = curr_best_params(2);
    l = curr_best_params(3:4);

end
