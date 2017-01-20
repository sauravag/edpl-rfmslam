function drawFeatureBelief(Pstar, fgrid)

% Plot the feature P
plot(fgrid,Pstar);
title('Normalized likelihood of feature 2 position in the world')
xlabel('Location');
ylabel('Likelihood');
grid on;
end