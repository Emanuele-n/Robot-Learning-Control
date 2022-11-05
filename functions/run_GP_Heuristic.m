function I = run_GP_Heuristic(training_set, I, active_p, sigma_n, sigma, l, offline_flag)

    global input_active output_active

    bar = waitbar(0, 'Heuristic performing ...');
    
    get_input_set = @(set) training_set(1:6, set);
    get_output_set = @(set) training_set(7:8, set);
    
    % heuristic insertion
    if isempty(I)
        I = zeros(active_p,1);
        
        indices = zeros(1,2);
        while indices(1) == indices(2)
            indices = randi(length(training_set),1,2);
        end
        I(1) = indices(1);
        I(2) = indices(2);

        % greedy insertions till I is full
        for i=3:active_p
            waitbar(i/active_p, bar);

            mu_I = Qval(get_input_set(I(1:i-1)), training_set(1:6,:), training_set(7:8,:), sigma_n, sigma, l, @RBF)';
            errors = mean(abs(mu_I - training_set(7:8,:)));

            % exclude indices already in I
            errors(1,I(1:i-1)) = 0;

            % pick the maximal error
            [~,index] = max(errors);
            I(i) = index;
        end
    end
    
    % heuristic update
    if ~offline_flag
        I_prev_sets = [];
        updates = 0;
        prev_index_added = 0;

        while 1

            I_set = get_input_set(I);
            Ki = RBF(I_set, I_set, sigma, l);
            L = chol(Ki + 1e-9*eye(length(Ki)), "lower"); %
            K_I_all = RBF(I_set, training_set(1:6,:), sigma, l);
            V = L\K_I_all;
            M = V*V' + sigma_n*eye(size(V,1));
            [Q,R] = qr(L * M * L');
            R = R + 1e-6*eye(length(R));
            alpha = R\(Q' * K_I_all * training_set(7:8,:)'); % size: training_p x n

            mu = Qval(I_set, training_set(1:6,:), training_set(7:8,:), sigma_n, sigma, l, @RBF)';
            errors = mean(abs(mu - training_set(7:8,:)));

            % exclude indices not in I
            errors_to_delete = errors(1,I);

            % pick the minimal value of deletion criterion
            [~,index_in_I] = min(abs(mean(alpha,2)' .* errors_to_delete));

            % exclude indices already in I
            errors(1,I) = 0;
            % pick the maximal error
            [~,index_to_add] = max(errors);

            if (prev_index_added ~= 0 && I(index_in_I) == prev_index_added) || (prev_index_added ~= 0 && ismember(I',I_prev_sets','rows'))
%                 if ismember(I',I_prev_sets','rows')
%                     disp('Index Set Already Encountered!');
%                 end
                disp("Heuristic Updated!");
                break
            end
            updates = updates + 1;

            % update index
            I_prev_sets = [I_prev_sets, I];
            I(index_in_I) = index_to_add;

            prev_index_added = index_to_add;

        end
    end
    
    close(bar);
    
    input_active = get_input_set(I);
    output_active = get_output_set(I);

end



