function run_GP_Initialization_Online(input, output)

    % - - - - - - - - - - - - - - - - - - -
    % input:
    % input: Inputs of the Data Set
    % output: Outputs of the Data Set
    % - - - - - - - - - - - - - - - - - - -
    % output (as global variables):
    % input_training: Training Points (x)
    % output_training: Training Points (y)
    % input_test: Test Points (x)
    % output_test: Test Points (y)
    % - - - - - - - - - - - - - - - - - - -

    global input_training output_training input_test output_test

    test_p = size(input,2)/5;
    dataset = [input;output];
    
    % Initialization
    steps = round(linspace(0,size(dataset,2),test_p+1));
    test_set = zeros(8, length(steps)-1);
    test_set_indices = zeros(1, length(steps)-1);
    
    for i=1:length(steps)-1
        
        % Sampled Normally
        step = steps(i+1) - (steps(i)+1);
        idx = round(normrnd((steps(i+1) + (steps(i)+1))/2, step*0.95/8));
        test_set(:,i) = dataset(:,idx);
        test_set_indices(i) = idx;

    end
    
    input_test = test_set(1:6,:);
    output_test = test_set(7:8,:);
    
    if size(input_training,2) >= size(input,2)*5
        
        input_training(:,1:size(input,2)) = [];
        output_training(:,1:size(input,2)) = [];
        
    end
    dataset(:,test_set_indices) = [];
    input_training = dataset(1:6,:);
    output_training = dataset(7:8,:);

end
