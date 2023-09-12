% direct run
% able to take both RGB and Grayscale image
function hw1_part2()
% img =  imread('cameraman-grayscale.jpg');
img = imread('campus.jpg');
[rol,col,n]=size(img);
x = reshape(img,rol*col,n);
k=5;
[means member] = my_kmeans(x,k,rol,col);
dimension = [size(means), size(member)] % displaying the dimension of means and member

function [means,member]=my_kmeans(x,k,rol,col)
%% plotting parameters
d_pos = [-0.02 -0.08 0.05 0.04];
supertitle = sgtitle('Program is running','fontweight','bold','FontSize',14,'Fontname','times new Roman');
%% section_1: kmeans clustering
n = size(x,2);
img_raw = reshape(x,rol,col,n);

if (n==3)
    mean_1 = mean(double(x(:,1)));
    mean_2 = mean(double(x(:,2)));
    mean_3 = mean(double(x(:,3)));

    if(mean_1 == mean_2 && mean_1 == mean_3) % if it is grayscale, all means shoule be equal
        img_lab = img_raw;
        text_1 = "(a).Raw input(Grayscale)";
        text_2 = "(b).Raw input(Grayscale)";
    else
        img_lab = rgb2lab(img_raw);
        text_1 = "(a).Raw input(RGB)";
        text_2 = "(b).Raw input(LAB)";
    end
else % if n = 1
    img_lab = img_raw;
    text_1 = "(a).Raw input(Grayscale)";
    text_2 = "(b).Raw input(Grayscale)";
end

subplot(3,3,1)
imshow(img_raw)
pos = get(gca, 'Position') ;
new_pos = pos +d_pos;
set(gca, 'Position',new_pos );
% daspect([1 1 1])
title(text_1);
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
set(gcf,'position',[100,50,1300,730]);

subplot(3,3,4)
imshow(img_lab)
pos = get(gca, 'Position') ;
new_pos = pos +d_pos;
set(gca, 'Position',new_pos );
title(text_2);
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
delete(supertitle)
supertitle = sgtitle('Program Paused, Press SPACE','fontweight','bold','FontSize',14,'Fontname','times new Roman');
pause();
delete(supertitle)
supertitle = sgtitle('Program is running','fontweight','bold','FontSize',14,'Fontname','times new Roman');
pause(0.1);

% [rol,col,~] = size(img_lab);
img_lab = double(img_lab);

% convergence criteria
tol = 1e-10;

%initialize random k points
rand_x = randperm(rol); % random permutation
rand_y = randperm(col);
x_sub = rand_x(1:k);
y_sub = rand_y(1:k);

%memory pre-allocation
means = zeros(k,n);

for j = 1:k
   means(j,:) = img_lab(x_sub(j),y_sub(j),:); 
end

means_update = zeros(k,n);

% kmeans iteration 
for i = 1:1000
    dist = zeros(rol*col, k);

    for j = 1: k
        img_temp = reshape(img_lab,rol*col,n);
        dist_temp = sum(sqrt((img_temp-means(j,:)).^2)'); % euclidean distance
        dist(:,j) = dist_temp;
    end

    [output_sort,idx] = sort(dist'); 
    member = idx(1,:);

    for j = 1:k
        idx_temp = find(member==j);
        img_reshape=reshape(img_lab,rol*col,n);
        means_update(j,:)=mean(img_reshape(idx_temp,:));
    end
    
    % if the difference is smaller than the tolerance
    if(max(abs(means-means_update))< tol)
        break;
    else
        means =means_update;
    end
    
    % results visualization
    pause(0.01);
    subplot(3,3,7)
    imagesc(reshape(member,rol,col));
    daspect([1 1 1])
    pos = get(gca, 'Position') ;
    new_pos = pos +d_pos;
    set(gca, 'Position',new_pos );
    axis off
    title(['(c).Clustering results, k = ',num2str(k),', iter = ',num2str(i)]);
    set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
end
delete(supertitle)
supertitle = sgtitle('Program Paused, Press SPACE','fontweight','bold','FontSize',14,'Fontname','times new Roman');
pause();
delete(supertitle)
supertitle = sgtitle('Program is Running','fontweight','bold','FontSize',14,'Fontname','times new Roman');
pause(0.1);
%% section_2: size merging (with loop)
%%% s_1 label each cluster individually and get rid of the connected
%%% components with size smaller than threshold (dim<30)
for i =1:k 
    % extract index of each clusters
    varName = ['lab_',num2str(i)];
    var = find(member==i);
    eval([varName '=var;']);
    
    % label one cluster as 1 per iteration
    img_bw = zeros(rol,col);
    img_bw(eval(['lab_',num2str(i)])) = 1;

    %connected component calculation
    lab_cc = bwconncomp(img_bw,8);
    groups = lab_cc.PixelIdxList;

%     %check the size of each components
%     for iter = 1:length(groups)
%         len_group(iter) = length(groups{iter});
%     end
%     plot([1:length(groups)],len_group) % for those smaller than 20 should be expelled
%     axis([0 300 0 200])

    subplot(3,3,2)
    imshow(img_bw);
    pos = get(gca, 'Position') ;
    new_pos = pos +d_pos;
    set(gca, 'Position',new_pos );
    title(['(d).ConnectedComponents in Cluster ',num2str(i),' = ',num2str(length(groups))]);
    set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
     
    % components pruning
    j = 0;
    for m = 1:(lab_cc.NumObjects)
        lab_temp = groups{m};
        if(length(groups{m})>30)
            j=j+1;
            img_bw(lab_temp) = 0.5; % set color to gray
        end
    end

    varName = ['idx_c',num2str(i)];
    var = find(img_bw ==0.5);
    eval([varName '=var;']);

    subplot(3,3,3)
    imshow(img_bw);
    pos = get(gca, 'Position') ;
    new_pos = pos +d_pos;
    set(gca, 'Position',new_pos );
    title(['(e).ConnectedComponent kept= ',num2str(j)]);
    set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
    delete(supertitle)
    supertitle = sgtitle('Program Paused, Press SPACE','fontweight','bold','FontSize',14,'Fontname','times new Roman');
    pause();
    delete(supertitle)
    supertitle = sgtitle('Program is Running','fontweight','bold','FontSize',14,'Fontname','times new Roman');
    pause(0.1);
end

%%% s_2 superposition all clusters together after pruning
%%% img_seg_1:segmentated image, have not deal with background yet
img_seg_1 = zeros(rol,col);

% labelling parameter
lab = 1/k;
for i = 1:k
    img_seg_1(eval(['idx_c',num2str(i)])) = i*lab;
end
% imshow(img_seg_1);

%%% s_3 dealing with background, pixels labelled by 0
%%% img_seg_2:segmentated image, with background pixels processed
subplot(3,3,5)
imshow(img_seg_1);
pos = get(gca, 'Position') ;
new_pos = pos +d_pos;
set(gca, 'Position',new_pos );
title('(f).Merged segement');
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
delete(supertitle)
supertitle = sgtitle('Program Paused, Press SPACE','fontweight','bold','FontSize',14,'Fontname','times new Roman');
pause();
delete(supertitle)
supertitle = sgtitle('Program is Running','fontweight','bold','FontSize',14,'Fontname','times new Roman');
pause(0.1);
img_seg_2 = img_seg_1;

r = 20; % sliding window size
for i = 1:rol
    for j = 1:col
        if(img_seg_2(i,j)==0)
            if(i-r<=0 && j+r<=col && j-r>0)
                img_seg_2(i,j) = mode(img_seg_2([1:i+r],[j-r:j+r]),'all');
            elseif(i-r<=0 && j-r<=0)
                img_seg_2(i,j) = mode(img_seg_2([1:i+r],[1:j+r]),'all');
            elseif(i-r<=0 && j+r>col)
                img_seg_2(i,j) = mode(img_seg_2([1:i+r],[j-r:col]),'all');
            elseif(i-r>0 && j+r>col && i+r<=rol)
                img_seg_2(i,j) = mode(img_seg_2([i-r:i+r],[j-r:col]),'all');
            elseif(j+r>col && i+r>rol)
                img_seg_2(i,j) = mode(img_seg_2([i-r:rol],[j-r:col]),'all');
            elseif(j+r<=col && i+r>rol && j-r>0)
                img_seg_2(i,j) = mode(img_seg_2([i-r:rol],[j-r:j+r]),'all');
            elseif(i+r>rol && j-r<=0)
                img_seg_2(i,j) = mode(img_seg_2([i-r:rol],[1:j+r]),'all');
            elseif(i+r<=rol && j-r<=0 && i-r>0)
                img_seg_2(i,j) = mode(img_seg_2([i-r:i+r],[1:j+r]),'all');
            else
                img_seg_2(i,j) = mode(img_seg_2([i-r:i+r],[j-r:j+r]),'all');
            end
        end
    end
end
subplot(3,3,6)
imshow(img_seg_2);
pos = get(gca, 'Position') ;
new_pos = pos +d_pos;
set(gca, 'Position',new_pos );
title('(g).SegmentatedImage PostProcessing');
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
delete(supertitle)
supertitle = sgtitle('Program Paused, Press SPACE','fontweight','bold','FontSize',14,'Fontname','times new Roman');
pause();
delete(supertitle)
supertitle = sgtitle('Program is Running','fontweight','bold','FontSize',14,'Fontname','times new Roman');
pause(0.1);

%%% s_3.2 members calculations
member = reshape(img_seg_2,rol*col,1)*k;

%%% s_4 obtain the boundary
edges_v = diff(img_seg_2,1,2); % vertical difference
edges_h = diff(img_seg_2,1,1); % horizontal difference
edges_h = [edges_h;zeros(1,col)];
idx_v = find(edges_v~=0);   % only label the nonzero value to be the boundary
idx_h = find(edges_h~=0);
edges = zeros(rol,col);
edges(idx_v) = 1;
edges(idx_h) = 1;
% imshow(edges);

img_edge = img_seg_2 - lab; 
img_edge(idx_v)=1;
img_edge(idx_h)=1;

subplot(3,3,8)
imshow(img_edge);
pos = get(gca, 'Position') ;
new_pos = pos +d_pos;
set(gca, 'Position',new_pos );
title('(h).Boundary Extraction');
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman') 
delete(supertitle)
supertitle = sgtitle('Program Paused, Press SPACE','fontweight','bold','FontSize',14,'Fontname','times new Roman');
pause();
delete(supertitle)
supertitle = sgtitle('Program Ends','fontweight','bold','FontSize',14,'Fontname','times new Roman');

%%% s_5 superposition the boundary on the raw image
img_raw_edge = reshape(double(img_raw),rol*col,n);
idx_edge = [idx_v' idx_h'];
img_raw_edge(idx_edge,:) = 255;
img_raw_edge=reshape(uint8(img_raw_edge),rol,col,n);
subplot(3,3,9)
imshow(img_raw_edge);
pos = get(gca, 'Position') ;
new_pos = pos +d_pos;
set(gca, 'Position',new_pos );
title('(i).Raw image with Boundaries');
set(gca,'fontsize',14,'fontweight','bold','Fontname','times new Roman');
