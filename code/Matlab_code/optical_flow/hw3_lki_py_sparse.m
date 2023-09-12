function hw3_lki_py_sparse()
im1 = imread('sphere1.jpg');
im2 = imread('sphere2.jpg'); 

% im1 = imread('frame10.png');
% im2 = imread('frame11.png');

% level of pyramid
level_p = 3;
py_image = py_show(im1,level_p);
% window size
ws = floor([size(im1,1) size(im1,2)]/10);

% optical flow
[u,v,harris,~] = my_lki_py_sparse(im1,im2,ws,level_p);

%%% vector transforming
vx_vector = -u; 
vy_vector = -v;
[x_grid, y_grid] = meshgrid(1:size(im1(:,:,1),2), 1:size(im1(:,:,1),1));

%%% plotting 
subplot(2,2,[1,3])
imshow(uint8(py_image))
title(['Image Pyramid, level=',num2str(level_p)])
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
set(gcf,'position',[200,50,1200,700]); 
subplot(2,2,2)
imshow(im1);
hold on;
j_harris = double(floor(harris.Location(:,1)));
i_harris = double(floor(harris.Location(:,2)));
plot(j_harris,i_harris,'r.','MarkerSize',10)
title(['Harris features, ',num2str(length(j_harris)),'Features'])
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 

subplot(2,2,4)
imshow(im1);
hold on;
% draw the velocity vectors
q = quiver(x_grid, y_grid, vx_vector,vy_vector, 'y','linewidth',0.5);
set(q,'AutoScaleFactor',20)
title('Vector Field')
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman')

% Harris feature & no iteration
function [vx,vy,harris,im1_r] = my_lki_py_sparse(im1,im2,ws,level_p)
scale_factor = fliplr(2.^([1:level_p]-1)); % scale factor for changing the window size for each layer
reduce_factor = 1./scale_factor;

im1_1 = im2double(rgb2gray(im1));
im2_1 = im2double(rgb2gray(im2));

% calculate speed for each level
for L = 1:level_p
    w_x = floor(round(ws(1)/2)/(scale_factor(L)));
    w_y = floor(round(ws(2)/2)/(scale_factor(L)));
    if(L<2)
        % downsize
        im1_r = imresize(im1_1, reduce_factor(L)); 
        im2_r = imresize(im2_1, reduce_factor(L));    

    else %L>2
        % downsize
        im1_r = imresize(im1_1, reduce_factor(L)); 
        im2_r = imresize(im2_1, reduce_factor(L));
        % resize speed from last time step to image of nect time step
        u_r = imresize(u_temp{L-1},size(im1_r)); 
        v_r = imresize(v_temp{L-1},size(im1_r));
        % warp the image according to the speed from previous time step
        im1_r = WarpImageBack(im1_r,u_r*scale_factor(L),v_r*scale_factor(L));
    end
    
    %%% detect harris features
    harris = detectHarrisFeatures(im1_r);
    j_harris = double(floor(harris.Location(:,1)));
    i_harris = double(floor(harris.Location(:,2)));
    
    vx_temp = zeros(size(im1_r));
    vy_temp = zeros(size(im1_r));
    iter = 0;
    % set the tolerance to be a large value for entering the while loop
    tol = 5;
    while iter < 20 && tol>3
        iter = iter+1;
        if(iter<2)
            [im1_dx,im1_dy] = gradient(im1_r);
            im_dt = im1_r - im2_r;
        else % iter>=2
            im1_warped = WarpImageBack(im1_r,vx_temp,vy_temp);
            [im1_dx,im1_dy] = gradient(im1_warped);
            % It_m = im1-im2;
            im_dt = im1_warped - im2_r;
        end
        vx = zeros(size(im1_r));
        vy = zeros(size(im1_r));
        %%% lucas kanade
        for i = 1:length(i_harris)
           % boundary check
           if(i_harris(i)-w_x<1)
               i_harris(i) = w_x+1;
           end
           if(i_harris(i)+w_x>size(im1_r,1))
               i_harris(i) =size(im1_r,1) - w_x;
           end
           if(j_harris(i)-w_y<1)
               j_harris(i) = w_y+1;
           end
           if(j_harris(i)+w_y>size(im1_r,2))
               j_harris(i) =size(im1_r,2) - w_y;
           end
           
           %%%only calculate speed for harris corners
           Ix = im1_dx(i_harris(i)-w_x:i_harris(i)+w_x, j_harris(i)-w_y:j_harris(i)+w_y);
           Iy = im1_dy(i_harris(i)-w_x:i_harris(i)+w_x, j_harris(i)-w_y:j_harris(i)+w_y);
           It = im_dt (i_harris(i)-w_x:i_harris(i)+w_x, j_harris(i)-w_y:j_harris(i)+w_y);
           b = -It(:); 
           A = [Ix(:) Iy(:)]; 
           
           % rank check and eigenvalue check
           A_2 = A(all(~isnan(A),2),:);
           b_2 = b(all(~isnan(A),2),:);
           A_3 = A_2(all(~isinf(A_2),2),:);
           b_3 = b(all(~isinf(A_2),2),:);
           if(rank(A_3'*A_3)==2 && abs(min(eig(A_3'*A_3)))>0.01)
               nu = pinv(A_3)*b_3; 
               vx(i_harris(i),j_harris(i))=nu(1);
               vy(i_harris(i),j_harris(i))=nu(2);
           else
               vx(i_harris(i),j_harris(i))=0;
               vy(i_harris(i),j_harris(i))=0;
           end
        end
        %%% update the speed per iteration
        vx_temp = vx_temp + vx;
        vy_temp = vy_temp + vy;
        tol = max([max(max(vx)) max(max(vy))]);
        %%% display the residuals
        res = [max(max(vx)) max(max(vy))]
    end
    % store the speed
    u_temp{L} = -vx_temp;
    v_temp{L} = -vy_temp;
end
% speed finalization 
u_f  =zeros(size(im1_1));
v_f  =zeros(size(im1_1));
for L = 1:level_p
    u_f = u_f + imresize(u_temp{L},size(im1_1))*scale_factor(L);
    v_f = v_f + imresize(v_temp{L},size(im1_1))*scale_factor(L);
end
%% warping
function [ I_warp ] = WarpImageBack( I,u,v )
[x, y] = meshgrid(1:size(I,2),1:size(I,1));
I_warp = interp2(I, x+0.7*u, y+0.7*v, 'cubic');
I_warp(isnan(I_warp)) = I(isnan(I_warp));

%% pyramid visualization
function [py_image] = py_show(im1,level_p)

scale_factor = fliplr(2.^([1:level_p]-1)); 
reduce_factor = 1./scale_factor;

% frame size
for i = 1:level_p
    im_temp = imresize(rgb2gray(im1), reduce_factor(i));
    size_row(i) = size(im_temp,1);
    pyramid{i} = im_temp;
end
size_col = size(im_temp,2);
py_image = zeros(sum(size_row),size_col);

for i = 1:level_p
    if (i==1)
        py_image(1:size(pyramid{i},1),[1:size(pyramid{i},2)]+floor(size_col/2-size(pyramid{i},2)/2))=pyramid{i};
    else
        py_image([1:size(pyramid{i},1)]+sum(size_row(1:i-1)),[1:size(pyramid{i},2)]+floor(size_col/2-size(pyramid{i},2)/2))=pyramid{i};
    end
end
