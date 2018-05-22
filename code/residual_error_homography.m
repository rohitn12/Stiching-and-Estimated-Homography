function residual = residual_error_homography(H, points1, points2);
transformed_points = points1 *H;
transformed_point_z = transformed_points(:,3);
x1 = transformed_points(:,1)./transformed_point_z;
x2 = points2(:,1)./points2(:,3);
y1 = transformed_points(:,2)./transformed_point_z;
y2 = points2(:,2)./points2(:,3);
dist1 = x1 - x2;
dist2 = y1- y2;
residual = (dist1).^2 + (dist2).^2;
end
