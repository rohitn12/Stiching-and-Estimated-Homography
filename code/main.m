%% load images and convert them to grayscale

img1 = imread('..\data\part1\uttower\left.jpg');
img2 = imread('..\data\part1\uttower\right.jpg');

img1 = im2double(img1);

img2 = im2double(img2);

gimg1 = rgb2gray(img1);
gimg2 = rgb2gray(img2);
[height1,width1] = size(gimg1);
[height2,width2] = size(gimg2);
%imshow(gimg2);

%% detect feature points

sigma = 2;
radius = sqrt(2) * sigma;
thresh = .005;

[cim1, r1, c1] = harris(gimg1,sigma,thresh,radius,0);
[cim2, r2, c2] = harris(gimg2,sigma,thresh,radius,0);

%% Extracting local neighborhood and keypoint 
kernel = 20;
featDescriptions_1 = localneighborhood(gimg1, kernel , r1, c1);

featDescriptions_2 = localneighborhood(gimg2, kernel , r2, c2);

%% matching 
thresholddistance = 200;
distances = dist2(featDescriptions_1, featDescriptions_2);
[row,col] = find(distances<thresholddistance);
fprintf('%d %d\n',size(row), size(col));


match_r1 = r1(row);
match_c1 = c1(row);
match_r2 = r2(col);
match_c2 = c2(col);

match_points_1 = [match_c1, match_r1, ones(length(match_c1),1)];
match_points_2 = [match_c2, match_r2, ones(length(match_c2),1)];

%% ransac
[best_model_h,best_inlier_count, best_inliers_indices ] = ransac(match_points_1,match_points_2);

%%

inliers_1 = match_points_1(best_inliers_indices,:);
inliers_2 = match_points_2(best_inliers_indices,:);
inliers_x1  = inliers_1(:,1);
inliers_y1 = inliers_1(:,2);
inliers_x2  = inliers_2(:,1);
inliers_y2 = inliers_2(:,2);

%fprintf('best model = %f', best_model_h);
fprintf('best model count = %d', best_inlier_count);

residual_avg = mean(residual_error_homography(best_model_h, inliers_1,inliers_2));

fprintf('best average residual = %f', residual_avg);


stitch_images(best_model_h, img1, img2);