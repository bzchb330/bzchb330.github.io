function hw4_main()
%% image reading
im{1} = imread('DSC_0554_s.jpg');
im{2} = imread('DSC_0555_s.jpg');
im{3} = imread('DSC_0556_s.jpg');
im{4} = imread('DSC_0557_s.jpg');
im{5} = imread('DSC_0558_s.jpg');

%% partition, thresholding & obtain harris features
level_p = 3; % pyramid level
m = 4; % must be even number
n = 2; 
[ind_total,tiles,tiles_bw,idx_h_local]=har_par_detect_bz(im{1},m,n );
par_g     = tiles;         % partition results gray scale
par_bw    = tiles_bw;      % partition results binary
idx_local = idx_h_local;   % local harris index
idx_global= ind_total;     % global harris index
i_harris  = ind_total(:,1);
j_harris  = ind_total(:,2);

u_mat = zeros(length(i_harris),level_p+1);
v_mat = zeros(length(i_harris),level_p+1);

i_harris_per = i_harris/size(im{1},1);
j_harris_per = j_harris/size(im{1},2);

u_mat(:,1) = i_harris_per;
v_mat(:,1) = j_harris_per;

%% plot partition results
% plotting partition results
idx_sub = [1:m*(n+2)];
idx_sub = reshape(idx_sub,(n+2),m)';
idx_sub_par = idx_sub(1:m,1:n)';
idx_sub_LK  = idx_sub(1:m,n+1:end);

% plot gray scale tiles
tiles_L = tiles;
par = 1;
for i_t = 1:m
    for j_t = 1:n
        h_par(par) = subplot(m,n+2,idx_sub_par(par));
        par = par + 1;
        imshow(uint8(tiles_L{i_t,j_t}))
        title(['[',num2str(i_t),', ',num2str(j_t),']']);
        axis off
        set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
        set(gcf,'position',[200,50,1200,700]);
        pause(0.01);
    end
end
pause();

% plot binary tiles
tiles_bw_L = tiles_bw;
par = 1;
for i_t = 1:m
    for j_t = 1:n
        delete(h_par(par))
        h_par(par) = subplot(m,n+2,idx_sub_par(par));
        par = par + 1;
        imshow(tiles_bw_L{i_t,j_t})
        title(['[',num2str(i_t),', ',num2str(j_t),']']);
        hold on
        h_local_idx = idx_h_local{i_t,j_t};
        plot(h_local_idx(:,1),h_local_idx(:,2), 'r.','MarkerSize',15),
        hold off
        axis off
        set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
        set(gcf,'position',[200,50,1200,700]);
        pause(0.01);
    end
end
pause();

idx_lk = idx_sub_LK(1:m/2,1:n/2);
subplot(m,n+2,idx_lk(:));
py_image = py_show(im{1},level_p);
imshow(uint8(py_image));
title(['Image Pyramid, Level =  ',num2str(level_p)])
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 

idx_lk = idx_sub_LK(m/2+1:m,1:n/2);
subplot(m,n+2,idx_lk(:));
imshow(im{1});
hold on;
plot(j_harris,i_harris,'y.','MarkerSize',10)
title(['There are, ',num2str(length(j_harris)),' Features'])
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
pause()

%% optical flow sparse
i_harris_temp = zeros(length(i_harris),length(im));
j_harris_temp = zeros(length(j_harris),length(im));
i_harris_temp(:,1) = i_harris;
j_harris_temp(:,1) = j_harris;
for idx = 1:length(im)-1
    % downsize factor 0~1
    rf = 1;
    ws = floor([size(im{idx},1) size(im{idx+1},2)]/15);
    
    u_mat = zeros(length(i_harris),level_p+1);
    v_mat = zeros(length(i_harris),level_p+1);

    i_harris_per = i_harris/size(im{1},1);
    j_harris_per = j_harris/size(im{1},2);

    u_mat(:,1) = i_harris_per;
    v_mat(:,1) = j_harris_per;
     
    % Harris feature & no iteration
    [u_mat,v_mat] = sparse_lki_py_bz(im{idx},im{idx+1},ws,u_mat,v_mat,level_p);
    scale_factor  = fliplr(2.^([1:level_p]-1));
    u_mat_f = u_mat;
    v_mat_f = v_mat;
    for L = 1:level_p
        u_mat_f(:,L+1) = u_mat_f(:,L+1)*scale_factor(L)*0.5;
        v_mat_f(:,L+1) = v_mat_f(:,L+1)*scale_factor(L)*0.5 ;
    end
    u_mat_f(:,2) = sum(u_mat_f(:,2:level_p+1),2);
    v_mat_f(:,2) = sum(v_mat_f(:,2:level_p+1),2);
    
    vx_vector = zeros([size(im{idx},1) size(im{idx+1},2)]);
    vy_vector = zeros([size(im{idx},1) size(im{idx+1},2)]);
    
    for i_h = 1:length(i_harris)
        vx_vector(i_harris(i_h),j_harris(i_h))= u_mat_f(i_h,2);
        vy_vector(i_harris(i_h),j_harris(i_h))= v_mat_f(i_h,2);
    end
    
    %%% vector transforming
    [m, n] = size(rgb2gray(im{idx}));
    [x_grid, y_grid] = meshgrid(1:n, 1:m);

    subplot(2,2,1)
    imshow(im{idx});
    hold on;
    plot(j_harris,i_harris,'y.','MarkerSize',10)
    title(['Harris features, ',num2str(length(j_harris)),'Features'])
    set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
    set(gcf,'position',[200,50,1200,700]);
    
    subplot(2,2,3)
    imshow(im{idx});
    hold on;
    % draw the velocity vectors
    q = quiver(x_grid, y_grid, vx_vector,vy_vector, 'y');
    set(q,'AutoScaleFactor',30)
    title('Vector Field')
    set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman')

    for i = 1:length(j_harris)
        if(vx_vector(i_harris(i),j_harris(i))<0)
            delta_j = floor(vx_vector(i_harris(i),j_harris(i)));
        else
            delta_j = ceil(vx_vector(i_harris(i),j_harris(i)));
        end

        if(vy_vector(i_harris(i),j_harris(i))<0)
            delta_i = floor(vy_vector(i_harris(i),j_harris(i)));
        else
            delta_i = ceil(vy_vector(i_harris(i),j_harris(i)));
        end
        j_harris2(i) = j_harris(i)+delta_j;
        i_harris2(i) = i_harris(i)+delta_i;
    end
    
    
    % get rid of the outsiders
    idx_j = find(j_harris2<1 | j_harris2>size(im{idx},2));
    idx_i = find(i_harris2<1 | i_harris2>size(im{idx},1));
    
    idx_ij = union(idx_j,idx_i);
   
    j_harris2(idx_ij) = [];
    i_harris2(idx_ij) = [];
    len(idx)=length(j_harris2);
    
    subplot(2,2,[2,4])
    imshow(im{idx+1});
    title(['Frame: ',num2str(idx+1)]);
    set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman')
    hold on;
    plot(j_harris2,i_harris2,'y.','MarkerSize',12)
    % plot lines
    j_harris = j_harris2;
    i_harris = i_harris2;
    i_harris_temp(:,idx+1) = i_harris';
    j_harris_temp(:,idx+1) = j_harris';
    for i_line = 1:2:length(j_harris)
        lines_tra(i_line) = line(j_harris_temp(i_line,1:idx+1),i_harris_temp(i_line,1:idx+1),...
            'Color','red','LineStyle','-','linewidth',2);
    end
    pause();
    if(idx<4)
        delete(lines_tra);
    end
end

%%% harris corner detection function 
%%% with image partition and thresholding
function [ind_total,tiles,tiles_bw,index_harris_local]=har_par_detect_bz(im_input,m,n)
if (length(size(im_input))==3)
    im_g = rgb2gray(im_input);
    im_g = double(im_g);
else
    im_g = double(im_input);
end

%% image partition

spacing_rol = floor(size(im_g,1)/m);
spacing_col = floor(size(im_g,2)/n);
k = 1;
for i = 1:m
    for j = 1:n
        vertices = [1+(j-1)*spacing_col 1+(i-1)*spacing_rol ...
                    spacing_col spacing_rol];
        tiles{i,j} = imcrop(im_g,vertices);
    end
end

%% gray scale thresholding 
for i = 1:m
    for j = 1:n
        % iteration based thresholding
        input = tiles{i,j};
        t = mean(mean(input));
        tol = 1e-6;
        for iter = 1:100
            mu_1 = mean(input(find(input<t)));
            mu_2 = mean(input(find(input>=t)));
            t_temp = 0.5*(mu_1+mu_2);
            if (abs(t_temp-t)<tol)
                break;
            else
                t = t_temp;
            end
        end
        % binarization 
        input(find(input<t))  = 0;
        input(find(input>=t)) = 255;
        tiles_bw{i,j}= input;
    end
end
%% Harris feature detection
sub_index=1;
for i_t = 1:m
    for j_t = 1:n
        gray = double(tiles_bw{i_t,j_t});
        % use builtin to detect harris feature
        % quality top 10 percent
        % distance to others 20 pixels
        harris = detectHarrisFeatures(gray,'MinQuality', 0.1,'FilterSize', 11);
        idx_all = randperm(length(harris.Location(:,1)));
        idx_use = idx_all(1:ceil(0.5*length(harris.Location(:,1))));
        [length(idx_all) length(idx_use)]
        i_col = double(floor(harris.Location(idx_use,1)));
        j_row = double(floor(harris.Location(idx_use,2)));
        
        index_harris_local{i_t,j_t} = [i_col j_row];
        sub_index=sub_index+1;
    end
end
%% index conversion 
%%% convert index from local to global
for i_t = 1:m
    for j_t = 1:n
        ind_local  = index_harris_local{i_t,j_t};
        col_global = ind_local(:,1)+(j_t-1)*spacing_col;
        rol_global = ind_local(:,2)+(i_t-1)*spacing_rol;
        index_harris_global{i_t,j_t} = [rol_global col_global];
        rol_global = [];
        col_global = [];
    end
end

%% combined feature plotting 
ind_total = [];
for i_t = 1:m
    for j_t = 1:n
        ind_global = index_harris_global{i_t,j_t};
        ind_total = [ind_total; ind_global];
    end
end

%%% optical flow with iteration and pyramid
function [u_mat,v_mat] = sparse_lki_py_bz(im1,im2,ws,u_mat,v_mat,level_p)
scale_factor  = fliplr(2.^([1:level_p]-1)); % scale factor for changing the window size for each layer
reduce_factor = 1./scale_factor;

im1_1 = im2double(rgb2gray(im1));
im2_1 = im2double(rgb2gray(im2));
u_mat_temp = zeros(size(u_mat,1),1);
v_mat_temp = zeros(size(v_mat,1),1);
% calculate speed for each level
for L = 1:level_p
    w_x = floor(round(ws(1)/2)/(scale_factor(L)));
    w_y = floor(round(ws(2)/2)/(scale_factor(L)));
    i_harris = floor(u_mat(:,1).*floor(size(im1_1,1)/(scale_factor(L))));
    j_harris = floor(v_mat(:,1).*floor(size(im1_1,2)/(scale_factor(L))));
    
    if(L<2)
        % downsize
        im1_r     = imresize(im1_1, reduce_factor(L),'bilinear'); 
        im2_r     = imresize(im2_1, reduce_factor(L),'bilinear');    
        im_out{L} = im1_r;
    else %L>2
        % downsize
        im1_r     = imresize(im1_1, reduce_factor(L),'bilinear'); 
        im2_r     = imresize(im2_1, reduce_factor(L),'bilinear');
        im_out{L} = im1_r;
        % resize speed from last time step to image of nect time step
        u_r = imresize(u_temp{L-1},size(im1_r),'bilinear'); 
        v_r = imresize(v_temp{L-1},size(im1_r),'bilinear');
        % warp the image according to the speed from previous time step
        im1_r = WarpImageBack(im1_r,u_r*scale_factor(L),v_r*scale_factor(L));
    end

    vx_temp = zeros(size(im1_r));
    vy_temp = zeros(size(im1_r));
    
    iter = 0;
    % set the tolerance to be a large value for entering the while loop
    tol = 20;
    while iter < 20 && tol>(12/scale_factor(L))
        iter = iter+1;
        if(iter<2)
            im1_dx = conv2(im1_r,[-1 1; -1 1], 'same'); 
            im1_dy = conv2(im1_r,[-1 -1; 1 1], 'same'); 
            im_dt  = conv2(im1_r, ones(2), 'same') + conv2(im2_r, -ones(2), 'same');
        else % iter>=2
            im1_warped = WarpImageBack(im1_r,vx_temp,vy_temp);
            im1_dx = conv2(im1_warped,[-1 1; -1 1], 'same'); 
            im1_dy = conv2(im1_warped,[-1 -1; 1 1], 'same'); 
            im_dt  = conv2(im1_warped, ones(2), 'same') + conv2(im2_r, -ones(2), 'same');
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

           %%%only perform calculation on harris corners
           Ix = im1_dx(i_harris(i)-w_x:i_harris(i)+w_x, j_harris(i)-w_y:j_harris(i)+w_y);
           Iy = im1_dy(i_harris(i)-w_x:i_harris(i)+w_x, j_harris(i)-w_y:j_harris(i)+w_y);
           It = im_dt (i_harris(i)-w_x:i_harris(i)+w_x, j_harris(i)-w_y:j_harris(i)+w_y);
           b = -It(:); 
           A = [Ix(:) Iy(:)]; 

           % rank check and eigenvalue check
           if(rank(A'*A)==2 && abs(min(eig(A'*A)))>0.05)
               nu = pinv(A)*b; 
               vx(i_harris(i),j_harris(i)) = nu(1);
               vy(i_harris(i),j_harris(i)) = nu(2);
               u_mat_temp(i) = nu(1);
               v_mat_temp(i) = nu(2);
           else
               vx(i_harris(i),j_harris(i)) = 0;
               vy(i_harris(i),j_harris(i)) = 0;
               u_mat_temp(i) = 0;
               v_mat_temp(i) = 0;
           end
        end
        %%% update the speed per iteration
        vx_temp = vx_temp + vx;
        vy_temp = vy_temp + vy;
        u_mat(:,L+1) = u_mat(:,L+1) + u_mat_temp;
        v_mat(:,L+1) = v_mat(:,L+1) + v_mat_temp;
        tol = max([max(max(vx)) max(max(vy))]);
        %%% display the residuals
        res = [max(max(vx)) max(max(vy))]
    end
    % store the speed
    u_temp{L} = vx_temp;
    v_temp{L} = vy_temp;
end

%%% warping function 
function [ I_warp ] = WarpImageBack( I,u,v )
[x, y] = meshgrid(1:size(I,2),1:size(I,1));
I_warp = interp2(I, x+0.9*u, y+0.9*v, 'cubic');
I_warp(isnan(I_warp)) = I(isnan(I_warp));

%% pyramid visualization
function [py_image] = py_show(im1,level_p)

scale_factor  = fliplr(2.^([1:level_p]-1)); 
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
