function Sigma_xx = marginalCovarianceUtility(Omega_xx, Omega_xy, Omega_yy)
% extract marginal covariance from Information Matrix
% the name is such to avoid conflict with gtsam
% Omega = [Omega_xx Omega_xy;...
%          Omega_yx Omega_yy];
% then 
% p(x) has Omega_xx_bar = Omega_xx - Omega_xy (Omega_yy)^-1 Omega_yx
% pg. 358, Probabilistic Robotics
% Basically Schur complement of Omega_yy
% Sigma_xx = inv(Omega_xx_bar)

% we use inverse function of TIM DAVIS which is awesome
% Omega_xx_bar = Omega_xx - Omega_xy*inverse(Omega_yy)*Omega_xy';
Omega_xx_bar = Omega_xx - Omega_xy*(Omega_yy\Omega_xy');

% here we use inv as x is a small dimensional variable
Sigma_xx = inv(Omega_xx_bar);
end