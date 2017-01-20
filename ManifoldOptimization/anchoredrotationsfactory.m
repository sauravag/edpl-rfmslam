function M = anchoredrotationsfactory(n, N, A, Ra)
% Returns M, a structure descripting the manifold P_A, which is a product
% of N times SO(n) (the rotations group) with the added constraint that
% anchors with indices in A are anchored (fixed) to the values in Ra (an
% n-by-n-by-length(A) matrix). P_A is a Riemannian submanifold of the
% manifold obtained via manopt.manifolds.rotations.rotationsfactory(n, N).
%
% Nicolas Boumal, UCLouvain, Nov. 23, 2012.

    import manopt.manifolds.rotations.*;
    
    Moriginal = rotationsfactory(n, N);
    M = Moriginal;
    
    if isempty(A)
        return;
    end

    strindices = sprintf('%d, ', A);
    strindices = strindices(1:end-2);
    M.name = @() [sprintf(['Product rotations manifold SO(%d)^%d with ' ...
                           'anchors at indices '], n, N) strindices];
    
    M.dim = @() (N-length(A))*nchoosek(n, 2);
    
    M.typicaldist = @() pi*sqrt(n*(N-length(A)));

    M.proj = @proj;
    function V = proj(X, U)
        V = Moriginal.proj(X, U);
        V(:, :, A) = 0;
    end

    M.tangent = @tangent;
    function V = tangent(X, U)
        V = Moriginal.tangent(X, U);
        V(:, :, A) = 0;
    end

    M.retr = @retraction;
    function Y = retraction(X, U, t)
        if nargin == 3
            Y = Moriginal.retr(X, U, t);
        else
            Y = Moriginal.retr(X, U);
        end
        Y(:, :, A) = Ra;
    end

    M.exp = @exponential;
    function Y = exponential(X, U, t)
        if nargin == 3
            Y = Moriginal.exp(X, U, t);
        else
            Y = Moriginal.exp(X, U);
        end
        Y(:, :, A) = Ra;
    end

    M.log = @logarithm;
    function U = logarithm(X, Y)
		U = Moriginal.log(X, Y);
        U(:, :, A) = 0;
    end

    M.rand = @random;
    function R = random()
        R = Moriginal.rand();
        R(:, :, A) = Ra;
    end

    M.randvec = @randomvec;
    function U = randomvec(X)
        U = Moriginal.randvec(X);
        U(:, :, A) = 0;
        U = U / Moriginal.norm(X, U);
    end

    M.vec = Moriginal.vec;
    M.mat = Moriginal.mat;

end