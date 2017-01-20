function problem = buildproblem(n, N, M, I, J, H, kappa, A, Ra)
% Returns a problem structure containing all information pertaining to a
% rotation synchronization problem.
%
% INPUTS:
%
%  n = 2, 3 or 4: work with rotations in SO(n), i.e., n-by-n orthog. matrices
%  N : number of rotations to synchronize
%  M : number of measurements of rotation ratios
%  I, J : length-M vectors of indices between 1 and N
%  H : n-by-n-by-M matrix; Each matrix H(:, :, k) in SO(n) is a measurement
%      of the ratio Ra*Rb', where Ra is the I(k)'s rotation matrix to
%      synchronize and Rb is the J(k)'s rotation matrix to synchronize
%  kappa1,2 : length-M vectors of confidences in the M measurements
%  p : probability that a measurement is not an outlier
%  A : (optional) vector of indices of the anchors of the problem, i.e.,
%      indices of the known rotation matrices; if omitted, replaced by [1].
%  Ra : (optional) nxnx|A| matrix with anchored rotations; if omitted,
%       replaced by the identity matrix eye(n).
% 
% OUTPUTS:
%
%  problem : a structure containing all the given information plus some
%            precomputed data.
%
% Nicolas Boumal, UCLouvain, Aug. 17, 2011
    
    problem = struct();
    
    problem.n = n;
    problem.N = N;
    problem.M = M;
    problem.I = I;
    problem.J = J;
    problem.H = H;
    problem.kappa = kappa;
    
    % date of birth of the problem structure
    problem.dob = clock();
    
    if exist('A', 'var') && ~isempty(A)
        problem.A = A;
        problem.Ra = Ra;
    else
        problem.A = 1;
        problem.Ra = eye(n);
    end
   
    [maskI, maskJ] = computemasks(N, I, J);
    problem.maskI = maskI;
    problem.maskJ = maskJ;
end