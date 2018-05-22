function[best_model_f,best_count_inliers, best_indices] = ransac_fundamental(total_matches_1, total_matches_2)
matches_count = length(total_matches_1);
iterations = 200;     
initial_match = 8;          
inlier_threshold = 0.1;  
num_inliers = zeros(iterations,1);
best_count_inliers = 0;

for i = 1:iterations
    %fprintf('running iterations\n');
    random_indices = randsample(matches_count, initial_match);
    select_points_1 = total_matches_1(random_indices, :);
    select_points_2 = total_matches_2(random_indices, :);
    ransac_matches = [select_points_1 select_points_2]; 
    model = fundamental_matrix(ransac_matches, 1);
    residualError = calculate_residuals_fundamental(model,[total_matches_1 total_matches_2]);
    inlier_indices = find(residualError < inlier_threshold);
    num_inliers(i) = length(inlier_indices);
    if num_inliers(i) > best_count_inliers
      %  fprintf('updated best model');
        best_count_inliers = num_inliers(i);
        best_model_f = model;
        best_indices = inlier_indices;
    end
end

end
