function [grad, store] = fungrad(problem, R, store)
%  grad = fungrad(problem, R)
% [grad store] = fungrad(problem, R, store)
%
% Computes the gradient of funcost at R.

    if ~exist('store', 'var')
        store = struct();
    end
    
    if isfield(store, 'egrad')
%         egrad = store.egrad;
        grad = store.grad;
        return;
    end    
    
    %% Extract data from problem structure.
    n = problem.n;
    M = problem.M;
    A = problem.A;
    H = problem.H;
    I = problem.I;
    J = problem.J;
    maskI = problem.maskI;
    maskJ = problem.maskJ;
    kappa = problem.kappa;
    
    
    %% Enforce the availability of data that the cost function is in
    %  charge of producing.
    if ~isfield(store, 'hatZ')
        [~, store] = funcost(problem, R, store);
    end
    
    hatZ = store.hatZ;
        
    %% Compute the gradient
    % Boumal had a fast version based on masking, use that
    g = kappa;
    
    g_hatZ = multiscale(g, hatZ);
    
    grad = zeros(size(R));
    
    % We write the code for the gradient in this way to avoid looping over
    % M elements, seen as M may be of order N^2 and Matlab doesn't like big
    % loops. An equivalent but much slower code is given below, for
    % readability.
    for k1 = 1 : n
        for k2 = 1 : n
            grad(k1, k2, :) = maskI * (squeeze(g_hatZ(k1, k2, :))) ...
                            - maskJ * (squeeze(g_hatZ(k1, k2, :)));
        end
    end
    
    grad = -grad / M;
    grad = .5*(grad - multitransp(grad));
    grad(:, :, A) = 0;
    
    % Store some data for the Hessian function.
    store.grad = grad;
    store.g = g;
    
    %% This is the slower version of the code for egrad
    
    %     egrad = zeros(size(R));   
%     for k = 1 : M
%         i = I(k);
%         j = J(k);
%         egrad(:, :, i) = egrad(:, :, i) + kappa(k)*H(:, :, k)*R(:,:,j);
%         egrad(:, :, j) = egrad(:, :, j) + kappa(k)*H(:, :, k)'*R(:,:,i);
%     end    
    
    % Normalize
%     egrad = -egrad / M;
%     egrad(:, :, A) = 0;    
    
    % Store some data for the Hessian function.
%     store.egrad = egrad;
end