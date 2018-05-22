function [ featDescriptions ] = localneighborhood( img, kernel, r, c )

featDescriptions = zeros((2 * kernel + 1)^2);
padding = zeros(ceil(2*kernel + 1));
padding(kernel + 1, kernel + 1) = 1;
paddedimg = imfilter(img, padding ,'replicate','full');
for i = 1 : length(r)
    row = r(i) : r(i) + (2*kernel);
    col = c(i) : c(i) + (2*kernel);
    neighborhood = paddedimg(row , col);
    flattenedvec = neighborhood( : );
    featDescriptions(i,:) = flattenedvec;
end
featDescriptions = zscore(featDescriptions')';
end     