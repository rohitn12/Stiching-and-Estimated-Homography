function stitch_images(h,img1, img2)

%% warping image
homography_transform = maketform('projective', h);
img1Transformed = imtransform(img1, homography_transform);
figure, imshow(img1Transformed);title('Warped image');

%% stitching images
img2_transform = maketform('affine', eye(3));
[~, x_data, y_data] = imtransform(img1, homography_transform, 'nearest');
x_range = [min(1, x_data(1)) max(size(img2, 2), x_data(2))];
y_range = [min(1, y_data(1)) max(size(img2, 1), y_data(2))];
img1_transformed = imtransform(img1, homography_transform, 'nearest', 'XData', x_range, 'YData', y_range);
img2_transformed = imtransform(img2, img2_transform, 'nearest', 'XData', x_range, 'YData', y_range);
d = img1_transformed;
final_image = img1_transformed;
for i = 1:numel(d)
    if (final_image(i) == 0)
        final_image(i) = img2_transformed(i);
    elseif (final_image(i) ~= 0 && img2_transformed(i) ~= 0)
        final_image(i) = (img1_transformed(i) + img2_transformed(i)) / 2;
    end
end
figure();
imshow(final_image);


end