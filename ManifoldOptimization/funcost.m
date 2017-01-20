function [cost, store] = funcost(problem, R, store)

    if ~exist('store', 'var')
        store = struct();
    end
    
    if isfield(store, 'cost')
        cost = store.cost;
        return;
    end    
    
    %% Extract data from the problem structure.
    n = problem.n;
    M = problem.M;
    H = problem.H;
    I = problem.I;
    J = problem.J;
    kappa = problem.kappa;
    
    %% Compute the cost
    Ri = R(:, :, I);
    Rj = R(:, :, J);
        
    hatZ = multiprod(multitransp(Ri), multiprod(H, Rj));
    
    trace_hatZ = multitrace(hatZ);
    
    cost = -sum(kappa.*(trace_hatZ-n))/M;
        
    %% Store some data for the gradient and the Hessian functions.
    store.cost = cost;
    store.hatZ = hatZ;

end