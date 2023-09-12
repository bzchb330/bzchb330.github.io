% direct run 
function hw1_part1_b()
I = imread('building-grayscale.jpg');  
I_2 = double(I(:,:,1));
[rol,col] = size(I_2);

input = reshape(double(I_2),rol*col,1); 
% histogram(input)
% [N,edges] = histcounts(input);
% bar(N,'histc')
for i=1:100
    subplot(1,3,1)
    imshow(I)
    title('Raw image input');
    set(gca,'fontsize',12,'fontweight','bold','Fontname','times new Roman') 

    subplot(1,3,2)
    histogram(input)
    title('Histogram of raw image input');
    sp(i) = sgtitle('LeftClick on Histogram to Continue, RightClick to End','fontweight','bold','FontSize',14,'Fontname','times new Roman');
    set(gca,'fontsize',12,'fontweight','bold','Fontname','times new Roman') 
    set(gcf,'position',[50,200,1400,400]); 
    [x,y,button]=ginput(1); % select a point on the histogram, left click only
    
    subplot(1,3,3)
    input2 = input;
    input2(find(input<x))=0;
    input2(find(input>=x))=255;
    imshow(reshape(input2,rol,col))
    title(['Thresholded image, with t = ',num2str(x)]);
    set(gca,'fontsize',12,'fontweight','bold','Fontname','times new Roman') 
    
    if(button==3) % if right click, end the loop
        break; 
    end  
end

delete(sp)
sgtitle('Program Ends','fontweight','bold','FontSize',14,'Fontname','times new Roman');
