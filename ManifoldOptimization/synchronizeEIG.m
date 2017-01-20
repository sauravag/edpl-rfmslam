function [R, flag] = synchronizeEIG(problem)
% Anchor-free synchronization of rotations with the eigenvector method.
% The measurements are weighted by the kappa1 given in the problem
% structure. The output R is a nxnxN matrix containing the estimated
% rotations.
%
% N. Boumal, A. Singer and P.-A. Absil, 2013,
%   Robust estimation of rotations from relative measurements
%   by maximum likelihood,
% In the proceedings of the 52nd Conference on Decision and Control (CDC).
%
% Nicolas Boumal, UCLouvain, 2013.
%
% See also: initialguess

    n = problem.n; % rotations in SO(n)
    N = problem.N; % N rotations to estimate
    M = problem.M; % M measurements available
    H = problem.H; % matrix of size nxnxM containing the measurements
    I = problem.I; % I and J are length-M vectors specifying the edges of
    J = problem.J; % measurement graph, that is, H(:, :, k) is a measurement
                   % of the rotation Ri*Rj^T with i = I(k) and j = J(k).
    kappa1 = problem.kappa; % weights of the measurements (length-M vector)
    
    % Synchronization matrix, made of nxn blocks corresponding to the
    % weighted measurements.
    W1 = spalloc(n*N, n*N, 2*n*n*M);
    
    % Weight vector: D(i) is the sum of the kappa1 weights of the edges
    % adjacent to node i.
    D = zeros(N, 1);
    
    % TODO: compute list of indices and populate sparse matrix all at once.
    for k = 1 : M
        
        i = I(k);
        j = J(k);
        weight = kappa1(k);
        
        W1( (i-1)*n + (1:n), (j-1)*n + (1:n) ) = weight*H(:, :, k);
        
        D(i) = D(i) + weight;
        D(j) = D(j) + weight;
    end
    
    W1 = W1 + W1';
    
    dd = repmat(D, 1, n).';
    D1 = spdiags(dd(:), 0, n*N, n*N);
    
    [X, E, flag] = eigs(W1, D1, n);
    
    % only flag = 0 means that all eigen values converged
    if flag ~= 0
        flag = -1;
    end
    
    % The eigenvectors of the (W1, D1) pencil, contained in X, do not
    % typically contain rotation matrices, because of the noise. Here, we
    % project the nxn blocks of X to the group of rotations. But we need to
    % choose whether we project X or X with its last eigenvector reversed.
    R1 = zeros(n, n, N);
    R2 = zeros(n, n, N);
    J = diag([ones(n-1, 1); -1]);
    for i = 1 : N
        Xi = X( (i-1)*n + (1:n) , :);
        R1(:, :, i) = soregister(Xi);
        R2(:, :, i) = soregister(Xi*J);
    end
    
    % Pick the most likely estimator among R1 and R2.
    % The J-operation is not tied to an invariance of the problem but
    % rather to the fact that the eigenvector method returns n eigenvectors
    % which can all be in one direction or another, independently from one
    % another. We may only choose to reverse one column (the last one for
    % example), and do so if the likelihood is improved by it.
    l1 = funcost(problem, R1);
    l2 = funcost(problem, R2);
    if l1 < l2
        R = R1;
    else
        R = R2;
    end
    
end
