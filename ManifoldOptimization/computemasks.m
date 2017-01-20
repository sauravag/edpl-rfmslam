function [maskI, maskJ] = computemasks(N, I, J)

    M = length(I);
    assert(length(J) == M);
    maskI = sparse(I, 1:M, ones(M, 1), N, M, M);
    maskJ = sparse(J, 1:M, ones(M, 1), N, M, M);
    
end