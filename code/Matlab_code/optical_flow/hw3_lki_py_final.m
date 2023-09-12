function hw3_lki_py_final()
im1 = imread('sphere1.jpg');
im2 = imread('sphere2.jpg'); 

% takes very long time
% im1 = imread('frame10.png');
% im2 = imread('frame11.png');

% level of pyramid
level_p = 3;
py_image = py_show(im1,level_p);
% window size
ws = floor([size(im1,1) size(im1,2)]/5);
% optical flow
[u,v,tolerance,iter] = my_lki_py(im1,im2,ws,level_p);

%%% vector transforming
vx_vector = -u(1:4:end, 1:4:end); 
vy_vector = -v(1:4:end, 1:4:end);
[x_grid, y_grid] = meshgrid(1:size(im1(:,:,1),2), 1:size(im1(:,:,1),1));

%%% plotting 
subplot(1,3,1)
imshow(uint8(py_image));
title(['Image Pyramid, level=',num2str(level_p)])
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
set(gcf,'position',[200,200,1150,400]); 

subplot(1,3,2)
imshow(im1);
hold on;
% draw the velocity vectors
q = quiver(x_grid(1:4:end, 1:4:end), y_grid(1:4:end, 1:4:end), vx_vector,vy_vector, 'y','linewidth',0.5);
set(q,'AutoScaleFactor',1.5)
title('Vector Field')
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman')
% max(max(vx_vector))
% max(max(vy_vector))
subplot(1,3,3)
% tolerance
plot([1:iter],tolerance(1:iter),'r','linewidth',2);
daspect([0.93 1 1])
xlabel('Iterations')
ylabel('Residuals')
title('Convergence plot')
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman')


%% pyramid loop
function [vx,vy,tolerance,iter] = my_lki_py(im1,im2,ws,level_p)
scale_factor = fliplr(2.^([1:level_p]-1)); % scale factor for changing the window size for each layer
reduce_factor = 1./scale_factor;

im1_g = im2double(rgb2gray(im1));
im2_g = im2double(rgb2gray(im2));

% calculate speed for each level
for L = 1:level_p
    w_x = floor(round(ws(1)/2)/(scale_factor(L)));
    w_y = floor(round(ws(2)/2)/(scale_factor(L)));
    % downsize
    im1_r = imresize(im1_g, reduce_factor(L)); 
    im2_r = imresize(im2_g, reduce_factor(L));
    if(L>2)
        % resize speed from last time step to image of nect time step
        u_r = imresize(u_temp{L-1},size(im1_r)); 
        v_r = imresize(v_temp{L-1},size(im1_r));
        % warp the image according to the speed from previous time step
        im1_r = WarpImageBack(im1_r,u_r*scale_factor(L),v_r*scale_factor(L));
    end
    
    %%% iterative refinement
    vx_temp = zeros(size(im1_r));
    vy_temp = zeros(size(im1_r));
    iter = 0;
    tol = 5;
    tolerance = zeros(6,1);
    while iter < 6 && tol>2
        iter = iter+1;
        if(iter<2)
            % image partial
            [im1_dx,im1_dy] = gradient(im1_r);
            % time difference
            im_dt = im1_r - im2_r;
        else % iter>=2
            im1_warped = WarpImageBack(im1_r,vx_temp,vy_temp);
            % image partial
            [im1_dx,im1_dy] = gradient(im1_warped);
             % time difference
            im_dt = im1_warped - im2_r;
        end
        
        %%% Lucas Kanade basic
        [vx, vy] = lk_basic(im1_dx,im1_dy,im_dt,w_x,w_y,zeros(size(im1_r)),zeros(size(im1_r)));
        % update velocity
        vx_temp = vx_temp + vx;
        vy_temp = vy_temp + vy;
        % calculate tolerance
        tol = max([max(max(vx)) max(max(vy))]);
        tolerance(iter) = tol;
    end
    % store the speed
    u_temp{L} = -vx_temp;
    v_temp{L} = -vy_temp;
end

% speed finalization 
u_f  =zeros(size(im1_g));
v_f  =zeros(size(im1_g));
for L = 1:level_p
    u_f = u_f + imresize(u_temp{L},size(im1_g))*scale_factor(L);
    v_f = v_f + imresize(v_temp{L},size(im1_g))*scale_factor(L);
end

%% lucas Kanade
function [vx, vy]=lk_basic(im1_dx,im1_dy,im_dt,w_x,w_y,vx,vy)
for i = w_x+1:size(im1_dx,1)-w_x
   for j = w_y+1:size(im1_dx,2)-w_y
      b = -reshape(im_dt(i-w_x:i+w_x, j-w_y:j+w_y),length(i-w_x:i+w_x)*length(j-w_y:j+w_y),1);    % It

      A = [reshape(im1_dx(i-w_x:i+w_x, j-w_y:j+w_y),length(i-w_x:i+w_x)*length(j-w_y:j+w_y),1)... % Ix
          reshape(im1_dy(i-w_x:i+w_x, j-w_y:j+w_y),length(i-w_x:i+w_x)*length(j-w_y:j+w_y),1)];   % Iy
      
      % get rid off nan and inf values
      A_2 = A(all(~isnan(A),2),:);
      b_2 = b(all(~isnan(A),2),:);
      A_3 = A_2(all(~isinf(A_2),2),:);
      b_3 = b(all(~isinf(A_2),2),:);
      
     % rank check and eigenvalue check
      if(rank(A_3'*A_3)==2 )
          nu = pinv(A_3)*b_3; 
          vx(i,j)=nu(1);
          vy(i,j)=nu(2);
      else
          vx(i,j)=0;
          vy(i,j)=0;
      end
   end
end
%% warping
function [ I_warp ] = WarpImageBack( I,u,v )
[x, y] = meshgrid(1:size(I,2),1:size(I,1));
I_warp = interp2(I, x+0.7*u, y+0.7*v, 'cubic');
I_warp(isnan(I_warp)) = I(isnan(I_warp));

%% visualize image pyramid
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