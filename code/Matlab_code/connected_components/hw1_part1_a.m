% direct run 
function hw1_part1_a()
d_pos = [-0.02 -0.04 0.05 0.04];
I = imread('mandrill-grayscale.jpg');
subplot(1,3,1);
imshow(I);
pos = get(gca, 'Position') ;
new_pos = pos +d_pos;
set(gca, 'Position',new_pos );
title('(A).Raw image input');
set(gca,'fontsize',12,'fontweight','bold','Fontname','times new Roman');
set(gcf,'position',[50,200,1400,400]); 

[rol,col,n] = size(I);
index_input = randperm(rol*col);
img_input = double(reshape(I(:,:,1),rol,col));
t_random = img_input(index_input(1));  % use random pixel values as input
t_mean = mean(mean(double(I(:,:,1)))); % use the overall mean of the image as input

[img_bw_r,t_r,i_r] = GrayscaleThresholding(I,t_random);
subplot(1,3,2);
imshow(img_bw_r)
pos = get(gca, 'Position') ;
new_pos = pos +d_pos;
set(gca, 'Position',new_pos );
title(['(B).threshold ti =',num2str(t_random), ', tf=',num2str(t_r), ', iteration:',num2str(i_r)]);
set(gca,'fontsize',12,'fontweight','bold','Fontname','times new Roman');

[img_bw_m,t_m,i_m] = GrayscaleThresholding(I,t_mean);
subplot(1,3,3);
imshow(img_bw_m)
pos = get(gca, 'Position') ;
new_pos = pos +d_pos;
set(gca, 'Position',new_pos );
title(['(C).threshold ti =',num2str(t_mean), ', tf=',num2str(t_m), ', iteration:',num2str(i_m)]);
set(gca,'fontsize',12,'fontweight','bold','Fontname','times new Roman');

function [img_bw,t,i] = GrayscaleThresholding(gray_img,t)
    I = gray_img(:,:,1);  
    I_2 = double(I);
%     imshow(I_2);
    [rol,col] = size(I_2);

    %% thresholding 
    input = reshape(double(I_2),rol*col,1); 
    tol = 1e-6;
    
    for i=1:10000
        mu_1 = mean(input(find(input<t)));
        mu_2 = mean(input(find(input>=t)));
        t_temp = 0.5*(mu_1+mu_2);
        if (abs(t_temp-t)<tol)
            break;
        else
            t=t_temp;
        end
    end

    img_bw = input;
    img_bw(find(input<t))=0;
    img_bw(find(input>=t))=255;
    img_bw = reshape(img_bw,rol,col);