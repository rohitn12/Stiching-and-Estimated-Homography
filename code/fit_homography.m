function [h] = fit_homography(select_points_1,select_points_2)
A = [];
for j = 1: length(select_points_1)
    point1 = select_points_1(j,:);
    point2 = select_points_2(j,:);
    A_j = [point1,0,0,0,(-point2(1))*point1;0,0,0,point1,(-point2(2))*point1];
    A = [A;A_j];
end
[~,~,V] = svd(A);
V = V(:,9);
h = reshape(V, [3 3]);
h = h./h(3,3);
end