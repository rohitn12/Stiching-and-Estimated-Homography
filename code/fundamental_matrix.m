function[F_final] = fundamental_matrix(matches,normalize)
x1 = matches(:,1:2);
x2 = matches(:,3:4);
points = size(matches,1);
if normalize == 1
    meanx1 = mean(x1);
    u1 = meanx1(1,1);
    v1 = meanx1(1,2);

    meanx2 = mean(x2);
    u2 = meanx2(1,1);
    v2 = meanx2(1,2);
    distance1 = sum((x1(:,1) - u1).^2+(x1(:,2) - v1).^2);
    distance2 = sum((x2(:,1) - u2).^2+(x2(:,2) - v2).^2);
    s1 = sqrt(2/(distance1/points));
    s2 = sqrt(2/(distance2/points));

    transform1 = [s1 0 0;0 s1 0;0 0 1] * [1 0 -u1;0 1 -v1;0 0 1];
    transform2 = [s2 0 0;0 s2 0;0 0 1] * [1 0 -u2;0 1 -v2;0 0 1];

    x1_t = zeros(points,2);
    x2_t = zeros(points,2);

    for i = 1:points

        pts1 = (transform1 * [x1(i,:) 1]')';
        x1_t(i,:) = pts1(1,1:2);

        pts2 = (transform2 * [x2(i,:) 1]')';
        x2_t(i,:) = pts2(1,1:2);
    end

    x1 = x1_t;
    x2 = x2_t;
end

u1 = x1(:,1);
v1 = x1(:,2);
u2 = x2(:,1);
v2 = x2(:,2);

A = [ u2.*u1 u2.*v1 u2 v2.*u1 v2.*v1 v2 u1 v1 ones(points, 1)];
[~, ~, V] = svd(A);
f = V(:,end);
F_final = reshape(f, [3 3])';
%% rank 2
[U, S, V] = svd(F_final);
S(3,3) = 0;
F_final= U*S*V';

if normalize == 1
    F_final = transform2' * F_final * transform1;
end


