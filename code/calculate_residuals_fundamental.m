function residual_fundamental = calculate_residuals_fundamental(F,matches)
points = size(matches,1);
L = (F * [matches(:,1:2) ones(points,1)]')';
L = L ./ repmat(sqrt(L(:,1).^2 + L(:,2).^2), 1, 3);
distances = sum(L .* [matches(:,3:4) ones(length(matches),1)],2);
residual_fundamental = abs(distances);
end
