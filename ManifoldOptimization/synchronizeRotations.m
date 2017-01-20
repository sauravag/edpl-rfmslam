function [Rmle, info] = synchronizeRotations(problem, R0, options)
% Solves the synchronization problem using a maximum likelihood approach.

    warning('off', 'manopt:getHessian:approx'); 
    
    if ~exist('problem', 'var') || ~isstruct(problem)
        error('The first input parameter (problem) must be a structure. See buildproblem.');
    end

    % Extract the problem parameters and anchors
    n = problem.n;
    N = problem.N;
    A = problem.A;
    Ra = problem.Ra;

    if ~exist('R0', 'var') || isempty(R0)
        R0 = initialguess(problem);
    end
    
    if ~exist('options', 'var')
        options = struct();
    end
    
    if ~isstruct(options)
        error('The third input parameter (options), if provided, must be a structure.');
    end
    
    % Specify default options here. The user supplied options structure
    % will complement (and possibly overwrite) the default options.
    % Note that maxiter is by default 1000
    default_options.maxinner = 200; % the number of inner Hessian computations per outer iteration
    default_options.maxiter = 100;
    default_options.tolgradnorm = 1e-12;
    default_options.debug = false;
    default_options.verbosity = 2;
    default_options.Delta_bar = 50;
     default_options.Delta0 = default_options.Delta_bar/12; 
    options = mergeOptions(default_options, options);
    
    % Obtain a Manopt description of the manifold: product of rotation
    % groups, with anchors indexed in A and given by Ra.
    manifold = anchoredrotationsfactory(n, N, A, Ra);
    optiproblem.M = manifold;

    % Specify the cost function and its derivatives. Notice that we use the
    % store caching capability of Manopt to cut down on redundant
    % computations.
    optiproblem.cost = @(R, store) funcost(problem, R, store);
    optiproblem.grad = @(R, store) fungrad(problem, R, store);
%     optiproblem.hess = @(R, store) funhess(problem, R, Omega, store);


    % For debugging purposes, it is nice to check numerically that the
    % gradient and the Hessian are compatible with the cost function.
%     figure; checkgradient(optiproblem);
%     figure; checkhessian(optiproblem);
    
    % The magic happens here: call the Riemannian trust-regions
    % optimization algorithm to maximize the likelihood (actually, to
    % minimize the opposite of the log-likelihood, normalized by the number
    % of measurements).
    [Rmle, ~, info] = trustregions(optiproblem, R0, options);

end