function[best_model_h,best_count_inliers, best_indices] = ransac(total_matches_1, total_matches_2)
matches_count = length(total_matches_1);
iterations = 200;     
initial_match = 4;          
inlier_threshold = 10;  
num_inliers = zeros(iterations,1);
best_count_inliers = 0;

for i = 1:iterations
    %fprintf('running iterations\n');
    random_indices = randsample(matches_count, initial_match);
    select_points_1 = total_matches_1(random_indices, :);
    select_points_2 = total_matches_2(random_indices, :);
    model = fit_homography(select_points_1,select_points_2);
    residualerror = residual_error_homography(model,total_matches_1,total_matches_2);
    inlier_indices = find(residualerror < inlier_threshold);
    num_inliers(i) = length(inlier_indices);
    if num_inliers(i) > best_count_inliers
        %fprintf('updated best model');
        best_count_inliers = num_inliers(i);
        best_model_h = model;
        best_indices = inlier_indices;
    end
end

end

    
        
    