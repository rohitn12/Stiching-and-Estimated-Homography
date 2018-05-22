%%
%% load images and match files for the first example
%%

I1 = imread('../data/part2/library1.jpg');
I2 = imread('../data/part2/library2.jpg');
matches = load('../data/part2/library_matches.txt'); 
% this is a N x 4 file where the first two numbers of each row
% are coordinates of corners in the first image and the last two
% are coordinates of corresponding corners in the second image: 
% matches(i,1:2) is a point in the first image
% matches(i,3:4) is a corresponding point in the second image

N = size(matches,1);

%%
%% display two images side-by-side with matches
%% this code is to help you visualize the matches, you don't need
%% to use it to produce the results for the assignment
%%
imshow([I1 I2]); hold on;
plot(matches(:,1), matches(:,2), '+r');
plot(matches(:,3)+size(I1,2), matches(:,4), '+r');
line([matches(:,1) matches(:,3) + size(I1,2)]', matches(:,[2 4])', 'Color', 'r');


%%
%% display second image with epipolar lines reprojected 
%% from the first image
%%

% first, fit fundamental matrix to the matches
F = fundamental_matrix(matches, 1); % this is a function that you should write

%mean(abs(calculate_residuals_fundamental(F,matches)))


[F, best_count_inliers, best_inlier_indices] = ransac_fundamental(matches(:,1:2),matches(:,3:4));
ransac_inliers = matches(best_inlier_indices,:);
L_ransac = (F * [ransac_inliers(:,1:2) ones(best_count_inliers,1)]')';
L_ransac = L_ransac ./ repmat(sqrt(L_ransac(:,1).^2 + L_ransac(:,2).^2), 1, 3);
pt_line_dist_ransac = sum(L_ransac .* [ransac_inliers(:,3:4) ones(best_count_inliers,1)],2);
mean(abs(pt_line_dist_ransac))
fprintf('%f',best_count_inliers);


L = (F * [matches(:,1:2) ones(N,1)]')'; % transform points from 
% the first image to get epipolar lines in the second image

% find points on epipolar lines L closest to matches(:,3:4)
L = L ./ repmat(sqrt(L(:,1).^2 + L(:,2).^2), 1, 3); % rescale the line
pt_line_dist = sum(L .* [matches(:,3:4) ones(N,1)],2);
closest_pt = matches(:,3:4) - L(:,1:2) .* repmat(pt_line_dist, 1, 2);

% find endpoints of segment on epipolar line (for display purposes)
pt1 = closest_pt - [L(:,2) -L(:,1)] * 10; % offset from the closest point is 10 pixels
pt2 = closest_pt + [L(:,2) -L(:,1)] * 10;

% display points and segments of corresponding epipolar lines
clf;
imshow(I2); hold on;
plot(matches(:,3), matches(:,4), '+r');
line([matches(:,3) closest_pt(:,1)]', [matches(:,4) closest_pt(:,2)]', 'Color', 'r');
line([pt1(:,1) pt2(:,1)]', [pt1(:,2) pt2(:,2)]', 'Color', 'g');


%% triangulation

cam_matrix1 = load('../data/part2/library1_camera.txt');
cam_matrix2 = load('../data/part2/library2_camera.txt');

x1 = matches(:,1:2);
x2 = matches(:,3:4);
points_count = size(matches,1);
triangulate_points = zeros(points_count, 3); % 3d
point_projection_img1 = zeros(points_count,2); %2d 
point_projection_img2 = zeros(points_count,2); %2d
 
for i = 1:points_count
    point1 = x1(i,:);
    point2 = x2(i,:);
    crossproductmatrix1 = [  0   -1  point1(2); 1   0   -point1(1); -point1(2)  point1(1)   0  ];
    crossproductmatrix2 = [  0   -1  point2(2); 1   0   -point2(1); -point2(2)  point2(1)   0  ]; 
    
    K = crossproductmatrix1 * cam_matrix1;
    L = crossproductmatrix2 * cam_matrix2;
    
    equation = [K ;L];
    [~,~,V] = svd(equation);
    tri_point = V(:,end)';
    triangulate_points(i,:) = tri_point(:,1:3)./tri_point(4);
    
    projection1 = (cam_matrix1*tri_point')';
    point_projection_img1(i,:) = projection1(:,1:2)./projection1(3); % 3d to 2d
    
    projection2 = (cam_matrix2*tri_point')';
    point_projection_img2(i,:) = projection2(:,1:2)./projection2(3); % 3d to 2d
    
end

%% get camera centers
[~,~,V] = svd(cam_matrix1);
camera1_center = V(:,end);

[~,~,V] = svd(cam_matrix2);
camera2_center = V(:,end);

plot_triangulation(triangulate_points,camera1_center,camera2_center);

%% get distances from projections
residual1 = 0;
residual2 = 0;
for i = 1 : points_count
    residual1 = residual1 + (x1(i,1) - point_projection_img1(i,1))^2 + (x1(i,2) - point_projection_img1(i,2))^2;
    residual2 = residual2 + (x2(i,1) - point_projection_img2(i,1))^2 + (x2(i,2) - point_projection_img2(i,2))^2;
end

fprintf('average residual1 = %f', residual1/points_count);
fprintf('\n');
fprintf('average residual1 = %f', residual2/points_count);


