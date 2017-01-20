function [R0, flag] = initialguess(problem)
% Computes an initial guess for a synchronization of rotations problem,
% based on the eigenvector method (synchronizeEIG). This function
% additionally takes anchors into account.
%
% The output R is a nxnxN matrix containing the estimated rotations.
%
% N. Boumal, A. Singer and P.-A. Absil, 2013,
%   Robust estimation of rotations from relative measurements
%   by maximum likelihood,
% In the proceedings of the 52nd Conference on Decision and Control (CDC).
%
% Nicolas Boumal, UCLouvain, Sept. 13, 2011.
%
% SEE ALSO: buildproblem synchronizeEIG synchronizeMLE

    % Obtain anchor information: A is the index list and Ra contains the
    % rotations associated to the anchors. Ra has size nxnxm, where is the
    % length of A, i.e., the number of anchors.
    A = problem.A;
    Ra = problem.Ra;
    
    tStart = tic;
    % The EIG method does not take anchors into account.
    [Reig, flag] = synchronizeEIG(problem);
    
    % Now align the EIG estimator with the anchors as well as possible,
    % then force the anchors.
    Q = soregister(Ra, Reig(:, :, A));
    R0 = multiprod(Reig, Q);
    R0(:, :, A) = Ra;

    tStop = toc(tStart);
    fprintf('Time to do Eigen value maximization for rotations: %f s \n', tStop)
end
