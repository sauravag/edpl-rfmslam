function Mout = nearPD(Min)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to fix non-pd covariance matrix
% This is a weird numerical issue which we handle.
% Finds the nearest PD matrix with a rank-1 update.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% We have arbitrarily chosen this value
TOL = 1e-14;

if issparse(Min)       
    error('Do not call nearPD for sparse matrix, remember it is never a good idea to invert a large sparse matrix!')
%     Mout = Min + TOL*speye(size(Min));
else
    [V,D] = eig(Min); % normal eig returns all eigen values
    
    % make vector of D
    Dv = diag(D);
    
    % Get the min eigen value
    minD = min(Dv);
    
    % If minD > 0 is not really small
    if minD >= TOL
        Mout = Min;
        return;
    end
    
    % Fix
    Mout = Min + (TOL+abs(minD))*eye(size(Min));
end

end