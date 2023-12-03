load testset.mat

path_fdm = part_outter(:,1:5);
path_laser = part_inner(:,1:5);
%% process fdm (coordinate extraction)
x = 0;
y = 0;
z = 0;
e = 0;
coord_fdm = zeros(length(path_fdm),4);

for i = 1:length(path_fdm)
    for j = 1:5
        if(path_fdm(i,j)== "")
            break
        else
            % c contains the coordinates
            % matches contains the coordinate letter
            [c,matches] = strsplit(path_fdm(i,j),'[A-Z]','DelimiterType','RegularExpression');
            if(matches=="X")
                x = str2num(c(2));
            elseif (matches=="Y")
                y = str2num(c(2));
            elseif (matches=="Z")
                z = str2num(c(2));
            elseif (matches=="E")
                e = str2num(c(2));
            end
        end
    end
    coord_fdm(i,:) = [x y z e];
end

%% center the pattern (FDM)
x_max = max(coord_fdm(floor(0.1*length(path_fdm)):end,1));
x_min = min(coord_fdm(floor(0.1*length(path_fdm)):end,1));
y_max = max(coord_fdm(floor(0.1*length(path_fdm)):end,2));
y_min = min(coord_fdm(floor(0.1*length(path_fdm)):end,2));
delta_x = 0.5*(x_max+x_min);
delta_y = 0.5*(y_max+y_min);
coord_fdm_center = coord_fdm;
coord_fdm_center(:,1) = coord_fdm_center(:,1) - delta_x;
coord_fdm_center(:,2) = coord_fdm_center(:,2) - delta_y;
hold on
% plot3(coord_fdm(:,1),coord_fdm(:,2),coord_fdm(:,3),'b')
plot3(coord_fdm_center(:,1),coord_fdm_center(:,2),coord_fdm_center(:,3),'b','linewidth',0.5)
daspect([1 1 1])
xlabel('x');
ylabel('y');
zlabel('z');
grid on
view(0,0);
view(0,90);
set(gca,'fontsize',28,'fontweight','bold','Fontname','times new Roman') 

%% process Laser (coordinate extraction)
x = 0;
y = 0;
z = 0;
e = 0;
f = 0; % need feedrate for control laser

coord_laser = zeros(length(path_laser),5);
for i = 1:length(path_laser)
    for j = 1:5
        if(path_laser(i,j)== "")
            break
        else
            % c contains the coordinates
            % matches contains the coordinate letter
            [c,matches] = strsplit(path_laser(i,j),'[A-Z]','DelimiterType','RegularExpression');
            if(matches=="X")
                x = str2num(c(2));
            elseif (matches=="Y")
                y = str2num(c(2));
            elseif (matches=="Z")
                z = str2num(c(2));
            elseif (matches=="E")
                e = str2num(c(2));
            elseif (matches=="F")
                f = str2num(c(2));
            end
        end
    end
    coord_laser(i,:) = [x y z e f];
end

%% center the pattern (laser)
x_max = max(coord_laser(:,1));
x_min = min(coord_laser(:,1));
y_max = max(coord_laser(:,2));
y_min = min(coord_laser(:,2));
delta_x = 0.5*(x_max+x_min);
delta_y = 0.5*(y_max+y_min);
coord_laser_center = coord_laser;

% % 2023 0919 center two path
coord_laser_center(:,1) = coord_laser_center(:,1) - delta_x+0.5;
coord_laser_center(:,2) = coord_laser_center(:,2) - delta_y-59.256; 

idx_x = find(abs(coord_laser_center(:,1))<1e-3);
idx_y = find(abs(coord_laser_center(:,2))<1e-3);
coord_laser_center(idx_x,1) = 0;
coord_laser_center(idx_y,2) = 0;
hold on
plot3(coord_laser_center(:,1),coord_laser_center(:,2),coord_laser_center(:,3),'r','linewidth',0.5)
% scatter3(coord_laser_center(1:100,1),coord_laser_center(1:100,2),coord_laser_center(1:100,3));
daspect([1 1 1])
xlabel('x');
ylabel('y');
zlabel('z');
grid on
view(0,0);
view(0,90);
set(gca,'fontsize',28,'fontweight','bold','Fontname','times new Roman') 
%% visualize points for switching on/off laser
idx_9600 = find(coord_laser_center(:,5)==9600);
hold on
plot3(coord_laser_center(:,1),coord_laser_center(:,2),coord_laser_center(:,3),'b');
scatter3(coord_laser_center(idx_9600,1),coord_laser_center(idx_9600,2),coord_laser_center(idx_9600,3),'r','linewidth',2);
daspect([1 1 1])
xlabel('x');
ylabel('y');
zlabel('z');
grid on
view(0,0);
view(0,90);
set(gca,'fontsize',28,'fontweight','bold','Fontname','times new Roman') 
%% process total layer numbers
plot(coord_fdm_center(:,3));
z_number = unique(coord_fdm_center(:,3));
z_number_laser = unique(coord_laser_center(:,3));
% plot(z_number)
layer_count = length(z_number);
layer_count_laser = length(z_number_laser);

%%% store toolpath of each layer(repeated lines has been removed) fdm part
coord_fdm_center_rep = coord_fdm_center(1,:);
k=1;
for i = 1:length(coord_fdm_center)
    if(min(coord_fdm_center_rep(k,:)==coord_fdm_center(i,:)))
    else
        k=k+1;
        coord_fdm_center_rep = [coord_fdm_center_rep; coord_fdm_center(i,:)];
    end
end
% plot3(coord_fdm_center_rep(:,1),coord_fdm_center_rep(:,2),coord_fdm_center_rep(:,3))

% %%% store toolpath of each layer(无重复的行) laser part
coord_laser_center_rep = coord_laser_center(1,:);
% coord_laser_center_rep = coord_laser_center;
k=1;
for i = 1:length(coord_laser_center)
    if(min(coord_laser_center_rep(k,:)==coord_laser_center(i,:)))
    else
        k=k+1;
        coord_laser_center_rep = [coord_laser_center_rep; coord_laser_center(i,:)];
    end
end
coord_laser_center = coord_laser_center_rep;
% plot3(coord_fdm_center_rep(:,1),coord_fdm_center_rep(:,2),coord_fdm_center_rep(:,3))

% fdm
for i = 1:layer_count
    idx_fdm_z = find(coord_fdm_center_rep(:,3)==z_number(i));
    coord_fdm_layer{i} = coord_fdm_center_rep(idx_fdm_z,:);
end
% laser
for i = 1:layer_count_laser
    idx_laser_z = find(coord_laser_center(:,3)==z_number_laser(i));
    coord_laser_layer{i} = coord_laser_center(idx_laser_z,:);
end

%%% adjust z number
% z_number_adjust, increase the layer height
spacing = 0.19;
% 0.19 layer height
z_number_2 = [z_number(1):spacing:z_number(end)];
z_number_adj = z_number_2;

%%% adjust z number
% z_number_adjust, increase the layer height
spacing = 0.15;
% 0.15 layer height
z_number_2 = [z_number(1):spacing:z_number(end)];
z_number_adj = z_number_2;
delta = 0.01;
z_delta = [delta:delta:delta*length(z_number_adj)]-delta;
z_number_adj = z_number_adj+z_delta;

%%% adjust z number
% z_number_adjust, increase the layer height
spacing = 0.12;
delta = 0.02;
z_delta = [delta:delta:delta*length(z_number)]-delta;
z_number_adj = z_number+z_delta';


%%% adjust z number
% z_number_adjust, increase the layer height
spacing = 0.1;
% 0.1 layer height
z_number_2 = [z_number(1):spacing:z_number(end)];
z_number_adj = z_number_2;
delta = 0.04;
z_delta = [delta:delta:delta*length(z_number_adj)]-delta;
z_number_adj = z_number_adj+z_delta;

%%% adjust z number
% z_number_adjust, increase the layer height
spacing = 0.2;
% 0.2 layer height
z_number_2 = [z_number(1):spacing:z_number(end)];
z_number_adj = z_number_2;
delta = 0.02;
z_delta = [delta:delta:delta*length(z_number_adj)];
z_number_adj = z_number_adj+z_delta;

%%% adjust z number
% z_number_adjust, increase the layer height
spacing = 0.3;
% 0.3 layer height
z_number_2 = [z_number(1):spacing:z_number(end)];
z_number_adj = z_number_2;
delta = 0.07;
z_delta = [delta:delta:delta*length(z_number_adj)];
z_number_adj = z_number_adj+z_delta;

%%% adjust z number
% z_number_adjust, increase the layer height
spacing = 0.5;
% 1mm layer height
z_number_2 = [z_number(1):spacing:z_number(end)];
z_number_adj = z_number_2;

%%% adjust z number
% z_number_adjust, increase the layer height
spacing = 0.05;
% 0.1 layer height
z_number_2 = [z_number(1):spacing:z_number(end)];
z_number_adj = z_number_2;
delta = 0.01;
z_delta = [delta:delta:delta*length(z_number_adj)]-delta;
z_number_adj = z_number_adj+z_delta;

%%% adjust z number (when z number is not evenly distributed)
z_number_adj = z_number';

%% z number adjustment
plot(coord_fdm_center(:,3));
z_number = unique(coord_fdm_center(:,3));
layer_count = length(z_number);
%%% store toolpath of each layer
% fdm
for i = 1:layer_count
    idx = find(coord_fdm_center(:,3)==z_number(i));
    coordinates_layer{i} = coord_fdm_center(idx,:);
end

%%% adjust z number
% z_number_adjust, increase the layer height
spacing = 0.19;
% 0.1 layer height
z_number_2 = [z_number(1):spacing:z_number(end)];
z_number_adj = z_number_2;

%% Gcode generation (FDM + laser) 
len = -1; % compensation length % updated by 2023/0113
len = -2;      % extruder compensation
len_2 = -0.10; % compensation each layer
for i = 1:layer_count
    coord_comp_layer_loc = coord_fdm_layer{i};
    coord_comp_layer_loc(1,:) = coord_comp_layer_loc(1,:) + [0 0 0 len];
    coord_comp_layer_loc(2,:) = coord_comp_layer_loc(2,:) + [0 0 0 len];
    coord_comp_layer_loc(3,:) = coord_comp_layer_loc(3,:) + [0 0 0 len];
    coord_comp_layer_loc(4,:) = coord_comp_layer_loc(4,:) + [0 0 0 len];
%     coord_comp_layer_loc(5,:) = coord_comp_layer_loc(5,:) + [0 0 0 len];


    if(i>1)
        coord_fdm_comp_layer{i} = coord_comp_layer_loc + [0 0 0 len_2*(i-1)]; %the first layer doesnt need to 
    else
        coord_fdm_comp_layer{i} = coord_comp_layer_loc;
    end
    coord_comp_layer_loc = [];
end

z_height = 21.7; % used to be 21.6
z_hop_value = 5;
power = 30;
fid = fopen('FileNameHere.txt','wt');
j=1;
Gcode_line(j,:) = string(['G10 L20 p1 X0 Y0 Z30 A0 B0 C0','\n']); %initial setup
fprintf(fid, Gcode_line(j,:));
j=j+1;
Gcode_line(j,:) = string(['G90','\n']);% set motion to absolute mode
fprintf(fid, Gcode_line(j,:));
laser_fdrt  = 200;

idx_9600 = find(coord_laser_center(:,5)==9600);
flag=0;

for i=1:layer_count
    %%%FDM starts
    coord_fdm_local = coord_fdm_comp_layer{i};
    coord_fdm_ref = coord_fdm_layer{i};
    if(i>1)
        for k = 3:size(coord_fdm_local,1)
            if(k==3) % move faster at the entry
                j=j+1; % only move no extruder when switching tools
                Gcode_line(j,:) = string(['F1400 G1 x',num2str(coord_fdm_local(2,1)),' y',num2str(-1*coord_fdm_local(2,2)),' z',num2str(z_number_adj(i)),'\n']);
                fprintf(fid, Gcode_line(j,:));
                j=j+1; % perform extrusion after reach location
                Gcode_line(j,:) = string(['F800 G1 c',num2str(coord_fdm_local(1,4)),'\n']);
                fprintf(fid, Gcode_line(j,:));
            else
                % printing code
                if(k<size(coord_fdm_local,1) &&... % get rid of repeat lines
                   coord_fdm_local(k+1,1)==coord_fdm_local(k,1) &&...
                   coord_fdm_local(k+1,2)==coord_fdm_local(k,2) &&...
                   coord_fdm_local(k+1,3)==coord_fdm_local(k,3))
                    if (coord_fdm_ref(k,4)==0)
                        j=j+1;
                        Gcode_line(j,:) = string(['G10 L20 p1 C0','\n']);
                        fprintf(fid, Gcode_line(j,:));
                    end
                else
                    if (coord_fdm_ref(k,4)==0)
                        j=j+1;
                        Gcode_line(j,:) = string(['G10 L20 p1 C0','\n']);
                        fprintf(fid, Gcode_line(j,:));
                    end
                    j=j+1;
                    Gcode_line(j,:) = string(['F800 G1 x',num2str(coord_fdm_local(k,1)),' y',num2str(-1*coord_fdm_local(k,2)),' z',num2str(z_number_adj(i)),' c',num2str(coord_fdm_local(k,4)),'\n']);
                    fprintf(fid, Gcode_line(j,:));
                end

                % retraction at the end of this layer
                if (k==size(coord_fdm_local,1)) % determine scratch direction
                    if(coord_fdm_local(k,1)>0)
                        delta = 4;
                    else
                        delta = -4;
                    end
                    % planar motion
                    Gcode_line(j,:) = string(['F1400 G1 x',num2str(coord_fdm_local(k,1)+delta),' y',num2str(-1*coord_fdm_local(k,2)),' z',num2str(z_number_adj(i)),' c',num2str(coord_fdm_local(k,4)),'\n']);
                    fprintf(fid, Gcode_line(j,:));
                    % retraction
                    j=j+1;
                    Gcode_line(j,:) = string(['F1400 G1 x',num2str(coord_fdm_local(k,1)+delta),' y',num2str(-1*coord_fdm_local(k,2)),' z',num2str(z_number_adj(i)),' c',num2str(coord_fdm_local(k,4)-20),'\n']);
                    fprintf(fid, Gcode_line(j,:));
                    % move nozzle higher to prevent collision
                    z_hop = z_hop_value;
                    j=j+1;
                    Gcode_line(j,:) = string(['F1400 G1 x',num2str(coord_fdm_local(k,1)+delta),' y',num2str(-1*coord_fdm_local(k,2)),' z',num2str(z_number_adj(i)+z_hop),' c',num2str(coord_fdm_local(k,4)-20),'\n']);
                    fprintf(fid, Gcode_line(j,:));
                end
            end 
        end %%%FDM ends
    else
        for k = 1:size(coord_fdm_local,1)
            if(k<=2) % move faster at the entry
                j=j+1; % only move no extruder when switching tools
                Gcode_line(j,:) = string(['F800 G1 x',num2str(coord_fdm_local(k,1)),' y',num2str(-1*coord_fdm_local(k,2)),' z',num2str(z_number_adj(i)),'\n']);
                fprintf(fid, Gcode_line(j,:));
                j=j+1; % perform extrusion after reach location
                Gcode_line(j,:) = string(['F800 G1 c',num2str(coord_fdm_local(k,4)),'\n']);
                fprintf(fid, Gcode_line(j,:));  
            else
                % printing code
                j=j+1;
                Gcode_line(j,:) = string(['F800 G1 x',num2str(coord_fdm_local(k,1)),' y',num2str(-1*coord_fdm_local(k,2)),' z',num2str(z_number_adj(i)),' c',num2str(coord_fdm_local(k,4)),'\n']);
                fprintf(fid, Gcode_line(j,:));

                % retraction at the end of this layer
                if (k==size(coord_fdm_local,1)) % determine scratch direction
                    if(coord_fdm_local(k,1)>0)
                        delta = 4;
                    else
                        delta = -4;
                    end
                    % 
                    Gcode_line(j,:) = string(['F1400 G1 x',num2str(coord_fdm_local(k,1)+delta),' y',num2str(-1*coord_fdm_local(k,2)),' z',num2str(z_number_adj(i)),' c',num2str(coord_fdm_local(k,4)),'\n']);
                    fprintf(fid, Gcode_line(j,:));
                    % 
                    j=j+1;
                    Gcode_line(j,:) = string(['F1400 G1 x',num2str(coord_fdm_local(k,1)+delta),' y',num2str(-1*coord_fdm_local(k,2)),' z',num2str(z_number_adj(i)),' c',num2str(coord_fdm_local(k,4)-20),'\n']);
                    fprintf(fid, Gcode_line(j,:));
                    % 
                    z_hop = z_hop_value;
                    j=j+1;
                    Gcode_line(j,:) = string(['F1400 G1 x',num2str(coord_fdm_local(k,1)+delta),' y',num2str(-1*coord_fdm_local(k,2)),' z',num2str(z_number_adj(i)+z_hop),' c',num2str(coord_fdm_local(k,4)-20),'\n']);
                    fprintf(fid, Gcode_line(j,:));
                end
            end 
        end 
    end %%%FDM ends
    
    %%% laser starts
%     member = [1:length(z_number_laser)];
    member = [2:2:24];
    if(ismember(i,member)) % only certian layer will be carbonized
        coord_laser_local = coord_laser_layer{i};
        coord_laser_local = coord_laser_local';
        % laser code
        j=j+1;
        Gcode_line(j,:) = string(['G90','\n']); % set motion to absolute mode
        fprintf(fid, Gcode_line(j,:));

        for m = 1:size(coord_laser_local,2)
            % variable Z height for carbonization
            if(mod(m,1)==0)
                z_height = 21.7;
            else
                z_height = 20.8;
            end
            
            if(m==1)% move faster when switching tools
                j=j+1;
                Gcode_line(j,:) = string(['F1000 G1 x',num2str(coord_laser_local(1,m)),' y',num2str(-1*coord_laser_local(2,m)),' z',num2str(z_height+z_number_adj(i)),'\n']);
                fprintf(fid, Gcode_line(j,:));
                
            elseif(m<=3)
                j=j+1;
                Gcode_line(j,:) = string(['F1000 G1 x',num2str(coord_laser_local(1,m)),' y',num2str(-1*coord_laser_local(2,m)),' z',num2str(z_height+z_number_adj(i)),'\n']);
                fprintf(fid, Gcode_line(j,:));
            elseif(m==4)
                j=j+1;
                Gcode_line(j,:) = string(['F1000 G1 x',num2str(coord_laser_local(1,m)),' y',num2str(-1*coord_laser_local(2,m)),' z',num2str(z_height+z_number_adj(i)),'\n']);
                fprintf(fid, Gcode_line(j,:));
                j=j+1;
                Gcode_line(j,:) = string(['m3 s5','\n']);  
                fprintf(fid, Gcode_line(j,:));
                j=j+1;
                Gcode_line(j,:) = string(['G4 P0.7','\n']);
                fprintf(fid, Gcode_line(j,:));
                j=j+1;
                Gcode_line(j,:) = string(['s',num2str(power),'\n']);  % switch laser on with (power/106)*12V
                fprintf(fid, Gcode_line(j,:));
                laser_fdrt = 200;
            else

               % method 2 switch off laser based on retraction speed
                if(coord_laser_local(5,m)==9600) 
                     j=j+1;
                    if (coord_laser_local(4,m)>coord_laser_local(4,m-1))
                        Gcode_line(j,:) = string(['s',num2str(power),'\n']);  % switch laser on with (power/106)*12V
                        laser_fdrt = 200;
                    else
                        Gcode_line(j,:) = string(['s5','\n']); 
                        laser_fdrt = 1200;
                    end
                    fprintf(fid, Gcode_line(j,:)); 
                end
                
                
                j=j+1;
                Gcode_line(j,:) = string(['F',num2str(laser_fdrt),' G1 x',num2str(coord_laser_local(1,m)),' y',num2str(-1*coord_laser_local(2,m)),' z',num2str(z_height+z_number_adj(i)),'\n']);
                fprintf(fid, Gcode_line(j,:));
            end
        end
        j=j+1;
        Gcode_line(j,:) = string(['m5','\n']);% switch off laser
        laser_fdrt = 200;
        fprintf(fid, Gcode_line(j,:));
    end
    %%% laser end
end

j=j+1;
Gcode_line(j,:) = string(['G90 x0 y0 z',num2str(30+max(z_number)), ' a0 b0','\n']);% go back to origin
fprintf(fid, Gcode_line(j,:));
fclose(fid)
