%% Load Images
I_left = imread('assignment3-1a.png');
I_right = imread('assignment3-1b.png');
I_left_2 = I_left;

[row,col,~] = size(I_left);
I_right = imresize(I_right,[row,col]);
I_right_2  = I_right;

% show original images
subplot(1,2,1),imshow(I_left),title('Left');
subplot(1,2,2),imshow(I_right),title('Right');

% change the format of images to single
I_left = single(rgb2gray(I_left)) ;
I_right = single(rgb2gray(I_right)) ;

% extract SIFT features by useing a package build-in function vl_sift
[f_l, d_l] = vl_sift(I_left) ;
[f_r, d_r] = vl_sift(I_right) ;

% find matched points by using a build-in function vl_ubcmatch (matches the two sets of SIFT descriptors)
[matches, score] = vl_ubcmatch(d_l, d_r) ;
matchPairs_l = f_l(:,matches(1,:));
matchPairs_r = f_r(:,matches(2,:));
% only need the first two cols to get matched points locations
matchPairs_l = matchPairs_l(1:2,:);
matchPairs_r = matchPairs_r(1:2,:);
% show the matched points
figure; ax = axes;
showMatchedFeatures(I_left, I_right, matchPairs_l', matchPairs_r','montage','Parent',ax);

%% RANSAC (random sample consesus)
k = 1;
while k<1000
    % Find 5 random points t ocalculate a homography matrix
    n = 5;
    index_n = randperm(length(matches));
    x1 = f_l(1:2,matches(1,:)); 
    x2 = f_r(1:2,matches(2,:)); 
    x1(3,:) = 1; % ensure homogenous condition
    x2(3,:) = 1;
    
    % Use corresponding points in both images to recover the parameters of the transformation
    % x_b, x_a --- x coordinates of point correspondences
    % y_b, y_a --- y coordinates of point correspondences
    a = zeros(2*n,9);
    for i = 1:n
        a_temp = [-x1(1,index_n(i)) -x1(2,index_n(i)) -1 0 0 0 x1(1,index_n(i))*x2(1,index_n(i)) x1(2,index_n(i))*x2(1,index_n(i)) x2(1,index_n(i));
                  0 0 0 -x1(1,index_n(i)) -x1(2,index_n(i)) -1 x1(1,index_n(i))*x2(2,index_n(i)) x1(2,index_n(i))*x2(2,index_n(i)) x2(2,index_n(i))];
        a(2*i-1:2*i,:) = a_temp; 
    end
    % Perform SVD computation on the assembled homography matrix
    [U,S,V] = svd(a);
    h = V(:,end);
    H = reshape(h,3,3)';

    x2_t = H * x1 ;                               % transformed features
    du = x2_t(1,:)./x2_t(3,:) - x2(1,:)./x2(3,:); % difference in x
    dv = x2_t(2,:)./x2_t(3,:) - x2(2,:)./x2(3,:); % difference in y
    reprojection = (du.*du + dv.*dv) < 36;        % (reprojection distance)^2
    score(k) = sum(reprojection);
    
    % Store the homography and inliers index with max inliers
    if(k>1) 
        if(score(k)>max(score(1:(k-1)  )))
            H_f = H;
            idx_inlier = find(reprojection==1);
        end
    else
        H_f = H;
        idx_inlier = find(reprojection==1);
    end
    k=k+1;
end

%% re-calculating homography matrix based on inliers
a = zeros(2*length(idx_inlier),9);
for i = 1:length(idx_inlier)
    a_temp = [-x1(1,idx_inlier(i)) -x1(2,idx_inlier(i)) -1 0 0 0 x1(1,idx_inlier(i))*x2(1,idx_inlier(i)) x1(2,idx_inlier(i))*x2(1,idx_inlier(i)) x2(1,idx_inlier(i));
               0 0 0 -x1(1,idx_inlier(i)) -x1(2,idx_inlier(i)) -1 x1(1,idx_inlier(i))*x2(2,idx_inlier(i)) x1(2,idx_inlier(i))*x2(2,idx_inlier(i)) x2(2,idx_inlier(i))];
        a(2*i-1:2*i,:) = a_temp;
end

% re-calculating H
[U,S,V] = svd(a);
h = V(:,end);
H = reshape(h,3,3)';

%% prepare the panorama
% set spatial reference position object for image to be warped
x_limit = [0 col];
y_limit = [0 row];
pos_Iwarp = imref2d(size(I_left_2),x_limit,y_limit);
% set spatial reference position object for reference image
x_limit = [0 col];
y_limit = [0 row];
pos_Iref = imref2d(size(I_right_2),x_limit,y_limit);

% apply Homography matrix to the image to be warped
[Iwarped, imref2d_inew] = imwarp(double(I_left_2),pos_Iwarp, projective2d(H'));

% create the canvas
rowVal_l = [round(imref2d_inew.YWorldLimits(1)) round(imref2d_inew.YWorldLimits(2))];
rowVal_r = [round(pos_Iref.YWorldLimits(1))   round(pos_Iref.YWorldLimits(2))];
colVal_l = [round(imref2d_inew.XWorldLimits(1)) round(imref2d_inew.XWorldLimits(2))];
colVal_r = [round(pos_Iref.XWorldLimits(1))   round(pos_Iref.XWorldLimits(2))];

xMin = min([rowVal_l rowVal_r]);
xMax = max([rowVal_l rowVal_r]);
yMin = min([colVal_l colVal_r]);
yMax = max([colVal_l colVal_r]);
panorama = zeros(abs(xMin)+abs(xMax),abs(yMin)+abs(yMax),3);

% Index of left image
index_iwarped_row = [min(rowVal_l)+abs(min(rowVal_l))+1:max(rowVal_l)+abs(min(rowVal_l))];
index_iwarped_col = [min(colVal_l)+abs(min(colVal_l))+1:max(colVal_l)+abs(min(colVal_l))];
% Index of right image
index_iref_row = [min(rowVal_r)+abs(min(rowVal_r))+abs(min(rowVal_l))+1:max(rowVal_r)+abs(min(rowVal_r))+abs(min(rowVal_l))];
index_iref_col = [min(colVal_r)+abs(min(colVal_r))+abs(min(colVal_l))+1:max(colVal_r)+abs(min(colVal_r))+abs(min(colVal_l))];

% stitching
panorama(index_iwarped_row,index_iwarped_col,:)=Iwarped;
panorama(index_iref_row,index_iref_col,:)      =I_right_2;

% show the final result
figure;
imshow(uint8(panorama))
