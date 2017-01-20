function [Q MSE unique] = soregister(R1, R2)
% [Q MSE] = soregister(R1, R2)
% 
% Input:
% 
%   R1: n-by-n-by-N matrix of N proper rotation matrices in SO(n).
%   R2: n-by-n-by-N matrix of N proper rotation matrices in SO(n).
%       If R2 is omitted, it is assumed to be all identity matrices,
%       and as a result Q is the mean of the matrices in R1.
%
% Output:
% 
%   Q: n-by-n proper rotation matrix in SO(n).
%   MSE: positive scalar value.
%   unique: boolean set at true if Q is the only solution, false otherwise.
%
%   Q and MSE correspond to a solution of this Frobenius norm minimization:
%
%   MSE = min_{Q \in SO(n)} (1/N)\sum_{i=1}^N ||R1(:,:,i)-R2(:,:,i)*Q||_F^2
% 
%   Q is uniquely defined in most cases.
%
%   A call to soregister(R2, R1) will return the same MSE but Q'.
% 
% This code is based on a paper by Sarlette and Sepulchre:
% Consensus Optimization on Manifolds, 2009
% This particular implementation in terms of an SVD is discussed in
% a coming paper whose provisional title is:
% Synchronization of rotations by maximum likelihood estimation
%
% Nicolas Boumal, UCLouvain, August 17, 2011.

    [n nbis N] = size(R1); %#ok<NASGU>
    assert(nbis == n);
    if nargin == 2
        assert(all(size(R1) == size(R2)));
    end

    if nargin == 2
        P = mean(multiprod(multitransp(R2), R1), 3);
    elseif nargin == 1
        P = mean(R1, 3);
    else
        error('Bad number of input arguments (should be 1 or 2).');
    end

    [U S V] = svd(P);
    UVt = U*V.';
    s = sign(det(UVt));
    if s > 0 % s == +1
        Q = UVt;
        unique = ( S(n-1, n-1) > 10*eps );
    else % s == -1
        Q = U*diag([ones(1, n-1) -1])*V.';
        unique = ( S(n-1, n-1)-S(n,n) > 10*eps );
    end
    
    MSE = 2*(n-[ones(1, n-1) s]*diag(S));
    
end
