%% recording purpose for recording.m
function stewart_platform(x_t,y_t,z_t,psi,theta,phi)
%% constant parameters
% Translation Vector
% x_t = input('Input Value for x_t: ');
% y_t = input('Input Value for y_t: ');
% z_t = input('Input Value for z_t: ');
% 
% % rotation angle (psi-z axis)
% psi   = input('Input Value for psi  : '); %(psi   - z axis)
% theta = input('Input Value for theta: '); %(theta - y axis)
% phi   = input('Input Value for z_t  : '); %(phi   - z axis)

% servo arm
a = 15;

% rod length
s = 85;

% rotation matrix (p means positive rotation)
R_z_p =[cosd(psi) -sind(psi) 0;
        sind(psi)  cosd(psi) 0;
          0         0        1];
      
R_y_p =[cosd(theta) 0 sind(theta);
         0          1   0   ;
       -sind(theta) 0 cosd(theta)];
   
R_x_p =[1   0          0;
        0  cosd(phi) -sind(phi);
        0  sind(phi)  cosd(phi)];

R_p_to_b=R_z_p * R_y_p * R_x_p;

% Bi coordinates

R_z_n_120 =[cosd(120) sind(120) 0; 
           -sind(120) cosd(120) 0;
             0         0        1];

x_b = 21;
y_b = 65;
z_b = 17.85;

b_1 = [ -x_b ; y_b ; z_b];
b_2 = [  x_b ; y_b ; z_b];
b_3 = R_z_n_120 * b_1;
b_4 = R_z_n_120 * b_2;
b_5 = R_z_n_120 * b_3;
b_6 = R_z_n_120 * b_4;

b_i = [b_1 b_2 b_3 b_4 b_5 b_6];

% b=[b_1 b_2 b_3 b_4 b_5 b_6];
% plot3(b(1,:),b(2,:),b(3,:),'O')
% view(0,90)
% daspect([1 1 1]);

% Pi coordinates
R_z_p_60 =[cosd(60) -sind(60) 0; %rotate p coordinates from p platform to base direction
           sind(60)  cosd(60) 0;
             0         0     1];
         
x_p = 21;
y_p = 55;
z_p = 0 ;

p_1 = R_z_p_60 * [-x_p ; y_p ; z_p]; % in frame p with orientated upward 
p_2 = R_z_p_60 * [ x_p ; y_p ; z_p];
p_3 = R_z_n_120 * p_1;
p_4 = R_z_n_120 * p_2;
p_5 = R_z_n_120 * p_3;
p_6 = R_z_n_120 * p_4;

% hold on
p=[p_2 p_3 p_4 p_5 p_6 p_1];
% plot3(p(1,:),p(2,:),p(3,:),'O','linewidth',2)
% view(0,90)
% daspect([1 1 1])

oleg        = p-b_i;
oleg_length = sqrt(sum((oleg).^2,1));

% home position, all rotation angles are zero
T_h = [0;0;90]; % in frame B
q_h_1 = T_h + p_2; %No.1 %platform 2 connect to base 1 therefore all offset 1 unit
q_h_2 = T_h + p_3; %No.2
q_h_3 = T_h + p_4; %No.3
q_h_4 = T_h + p_5; %No.4
q_h_5 = T_h + p_6; %No.5
q_h_6 = T_h + p_1; %No.6

q_h = [q_h_1 q_h_2 q_h_3 q_h_4 q_h_5 q_h_6];

% desired position of platform, beased on variables input
T   = [ x_t ; y_t ; z_t]; % in frame B
q_1 = T + R_p_to_b * p_2; % in frame B
q_2 = T + R_p_to_b * p_3;
q_3 = T + R_p_to_b * p_4;
q_4 = T + R_p_to_b * p_5;
q_5 = T + R_p_to_b * p_6;
q_6 = T + R_p_to_b * p_1;

q_i = [q_1 q_2 q_3 q_4 q_5 q_6];

% li calculation, li:length of i_th leg  li = T + R_p_to_b * pi - bi
li        = q_i-b_i;
li_square = sum((li).^2,1); % sum elements in each column

%% Inverse Kinematics
%calculation of L = li2-(s2-a2)
L     = li_square-(s^2-a^2)*ones(1,6);
M     = 2*a*(q_i(3,:)-b_i(3,:));
beta  = [0 180 240 60 120 300]; 
N     = 2*a*(cosd(beta).*(q_i(1,:)-b_i(1,:))+sind(beta).*(q_i(2,:)-b_i(2,:)));
alpha = asind(L./sqrt(M.^2+N.^2))-atand(N./M) 


x_a_1 = a*cosd(    alpha(1))*cosd(    beta(1)) + b_i(1,1);
x_a_2 = a*cosd(180-alpha(2))*cosd(180+beta(2)) + b_i(1,2);
x_a_3 = a*cosd(    alpha(3))*cosd(    beta(3)) + b_i(1,3);
x_a_4 = a*cosd(180-alpha(4))*cosd(180+beta(4)) + b_i(1,4);
x_a_5 = a*cosd(    alpha(5))*cosd(    beta(5)) + b_i(1,5);
x_a_6 = a*cosd(180-alpha(6))*cosd(180+beta(6)) + b_i(1,6);

y_a_1 = a*cosd(    alpha(1))*sind(    beta(1)) + b_i(2,1);
y_a_2 = a*cosd(180-alpha(2))*sind(180+beta(2)) + b_i(2,2);
y_a_3 = a*cosd(    alpha(3))*sind(    beta(3)) + b_i(2,3);
y_a_4 = a*cosd(180-alpha(4))*sind(180+beta(4)) + b_i(2,4);
y_a_5 = a*cosd(    alpha(5))*sind(    beta(5)) + b_i(2,5);
y_a_6 = a*cosd(180-alpha(6))*sind(180+beta(6)) + b_i(2,6);

z_a_1 = a*sind(    alpha(1)) + b_i(3,1);
z_a_2 = a*sind(180-alpha(2)) + b_i(3,2);
z_a_3 = a*sind(    alpha(3)) + b_i(3,3);
z_a_4 = a*sind(180-alpha(4)) + b_i(3,4);
z_a_5 = a*sind(    alpha(5)) + b_i(3,5);
z_a_6 = a*sind(180-alpha(6)) + b_i(3,6);

a_1 = [x_a_1;y_a_1;z_a_1];
a_2 = [x_a_2;y_a_2;z_a_2];
a_3 = [x_a_3;y_a_3;z_a_3];
a_4 = [x_a_4;y_a_4;z_a_4];
a_5 = [x_a_5;y_a_5;z_a_5];
a_6 = [x_a_6;y_a_6;z_a_6];

servo_a=[a_1 a_2 a_3 a_4 a_5 a_6];
plot3(servo_a(1,:),servo_a(2,:),servo_a(3,:),'O')
view(0,90)
daspect([1 1 1])
%% plot part
red       =[255,0,0]/255;
blue      =[0,0,255]/255;
green     =[0,255,0]/255;
light_blue=[124,106,240]/255;


% actual link length
oleg_1=[a_1 q_1];
oleg_2=[a_2 q_2];
oleg_3=[a_3 q_3];
oleg_4=[a_4 q_4];
oleg_5=[a_5 q_5];
oleg_6=[a_6 q_6];

arm_1=[a_1 b_1];
arm_2=[a_2 b_2];
arm_3=[a_3 b_3];
arm_4=[a_4 b_4];
arm_5=[a_5 b_5];
arm_6=[a_6 b_6];

% link length equivalent to linear motor
rod_1=[b_1 q_1];
rod_2=[b_2 q_2];
rod_3=[b_3 q_3];
rod_4=[b_4 q_4];
rod_5=[b_5 q_5];
rod_6=[b_6 q_6];

hold on
%platform
fill3(q_i(1,:),q_i(2,:),q_i(3,:),light_blue);
%arm
plot3(arm_1(1,:),arm_1(2,:),arm_1(3,:),'b','linewidth',2);
plot3(arm_2(1,:),arm_2(2,:),arm_2(3,:),'b','linewidth',2);
plot3(arm_3(1,:),arm_3(2,:),arm_3(3,:),'b','linewidth',2);
plot3(arm_4(1,:),arm_4(2,:),arm_4(3,:),'b','linewidth',2);
plot3(arm_5(1,:),arm_5(2,:),arm_5(3,:),'b','linewidth',2);
plot3(arm_6(1,:),arm_6(2,:),arm_6(3,:),'b','linewidth',2);

%rod
plot3(oleg_1(1,:),oleg_1(2,:),oleg_1(3,:),'r','linewidth',2);
plot3(oleg_2(1,:),oleg_2(2,:),oleg_2(3,:),'r','linewidth',2);
plot3(oleg_3(1,:),oleg_3(2,:),oleg_3(3,:),'r','linewidth',2);
plot3(oleg_4(1,:),oleg_4(2,:),oleg_4(3,:),'r','linewidth',2);
plot3(oleg_5(1,:),oleg_5(2,:),oleg_5(3,:),'r','linewidth',2);
plot3(oleg_6(1,:),oleg_6(2,:),oleg_6(3,:),'r','linewidth',2);

stewart_platform_base_plot()

axis([-100 100 -100 100 -10 125]);
grid on
daspect([1 1 1]);
xlabel('x');
ylabel('y');
zlabel('z');
view(-10,25);
hold off
% Operating Leg length calculation
L_oleg_1 = sqrt(sum((oleg_1(:,1)-oleg_1(:,2)).^2));
L_oleg_2 = sqrt(sum((oleg_2(:,1)-oleg_2(:,2)).^2));
L_oleg_3 = sqrt(sum((oleg_3(:,1)-oleg_3(:,2)).^2));
L_oleg_4 = sqrt(sum((oleg_4(:,1)-oleg_4(:,2)).^2));
L_oleg_5 = sqrt(sum((oleg_5(:,1)-oleg_5(:,2)).^2));
L_oleg_6 = sqrt(sum((oleg_6(:,1)-oleg_6(:,2)).^2));
L_oleg   = [L_oleg_1 L_oleg_2 L_oleg_3 L_oleg_4 L_oleg_5 L_oleg_6];

% arm length calculation
L_arm_1 = sqrt(sum((arm_1(:,1)-arm_1(:,2)).^2));
L_arm_2 = sqrt(sum((arm_2(:,1)-arm_2(:,2)).^2));
L_arm_3 = sqrt(sum((arm_3(:,1)-arm_3(:,2)).^2));
L_arm_4 = sqrt(sum((arm_4(:,1)-arm_4(:,2)).^2));
L_arm_5 = sqrt(sum((arm_5(:,1)-arm_5(:,2)).^2));
L_arm_6 = sqrt(sum((arm_6(:,1)-arm_6(:,2)).^2));
L_arm   = [L_arm_1 L_arm_2 L_arm_3 L_arm_4 L_arm_5 L_arm_6];

% rod length calculation
L_rod_1 = sqrt(sum((rod_1(:,1)-rod_1(:,2)).^2));
L_rod_2 = sqrt(sum((rod_2(:,1)-rod_2(:,2)).^2));
L_rod_3 = sqrt(sum((rod_3(:,1)-rod_3(:,2)).^2));
L_rod_4 = sqrt(sum((rod_4(:,1)-rod_4(:,2)).^2));
L_rod_5 = sqrt(sum((rod_5(:,1)-rod_5(:,2)).^2));
L_rod_6 = sqrt(sum((rod_6(:,1)-rod_6(:,2)).^2));
L_rod   = [L_rod_1 L_rod_2 L_rod_3 L_rod_4 L_rod_5 L_rod_6];
return

function stewart_platform_base_plot()
%% platform parameters
red       =[255,0,0]/255;
blue      =[73,79,232]/255;
green     =[35,255,49]/255;
light_blue=[124,106,240]/255;
dark      =[27,31,113]/255;

xp_16   = 60;
yp_16   = 21;
yA      = 34.641;
theta_p = 120;

P_1 = [-xp_16 ; yp_16 ; 0];
P_6 = [-xp_16 ;-yp_16 ; 0];
A_p = [-xp_16 ; yA    ; 0];
F_p = [-xp_16 ;-yA    ; 0];

R_z =[cosd(theta_p) sind(theta_p) 0;
     -sind(theta_p) cosd(theta_p) 0;
      0              0            1];
  
R_z_30 =[cosd(30) sind(30) 0;
        -sind(30) cosd(30) 0;
         0        0        1];

P_3 = R_z*P_1;
P_2 = R_z*P_6;
P_4 = R_z*P_2;
P_5 = R_z*P_3;

C_p = R_z*A_p;
B_p = R_z*F_p;
D_p = R_z*B_p;
E_p = R_z*C_p;

point_platform = [A_p B_p C_p D_p E_p F_p A_p];
point_hole     = [P_1 P_2 P_3 P_4 P_5 P_6];

point_platform=R_z_30*point_platform;

% plot3(point_platform(1,:),point_platform(2,:),point_platform(3,:),'r','linewidth',2);
% hold on
% scatter(point_hole(1,:),point_hole(2,:),point_hole(3,:));
hold on
fill3(point_platform(1,:),point_platform(2,:),point_platform(3,:),dark);

x_end_h=[-100 100];
y_end_h=[0 0];
z_end_h=[0 0];

x_end_v=[0 0];
y_end_v=[-100 100];
z_end_v=[0 0];

x_end_r=[0 0];
y_end_r=[0 0];
z_end_r=[0 200];

plot3(x_end_h,y_end_h,z_end_h,'m--',x_end_v,y_end_v,z_end_v,'m--',x_end_r,y_end_r,z_end_r,'m--','linewidth',1);
% axis([-100 100 -100 100 -10 200]);
% grid on
% daspect([1 1 1]);

%% servo motor
% 舵机转动轴长10.95 舵机body 长24 宽12 高16.5

alph=0:0.01:2*pi;
R1   = 1;
R2   = 1.1;

x1   = R1*cos(alph);
y1   = R1*sin(alph);
z1   = zeros(1,length(x1));

x2   = R2*cos(alph);
y2   = R2*sin(alph);
z2   = zeros(1,length(x2));

circle1   = [x1;y1;z1];
circle2   = [x2;y2;z2];

R_x_posi = [1  0              0;
            0  cosd(90) -sind(90);
            0  sind(90)  cosd(90)];
        
circle1_v_s1=R_x_posi*circle1;

circle2_v_s1=R_x_posi*circle2-[zeros(1,length(alph));10.95*ones(1,length(alph));zeros(1,length(alph))];

circle1_v_s1= circle1_v_s1+[-21*ones(1,length(alph));65*ones(1,length(alph));17.85*ones(1,length(alph))] ;
circle2_v_s1= circle2_v_s1+[-21*ones(1,length(alph));65*ones(1,length(alph));17.85*ones(1,length(alph))] ;


% pillar of servo
fill3(circle1_v_s1(1,:),circle1_v_s1(2,:),circle1_v_s1(3,:),red);
fill3(circle2_v_s1(1,:),circle2_v_s1(2,:),circle2_v_s1(3,:),blue);
fill3([circle1_v_s1(1,:) fliplr(circle2_v_s1(1,:))],...
      [circle1_v_s1(2,:) fliplr(circle2_v_s1(2,:))],...
      [circle1_v_s1(3,:) fliplr(circle2_v_s1(3,:))],blue);
axis equal
% xlabel('x');
% ylabel('y');
% grid on

R_z =[cosd(90) sind(90) 0;
     -sind(90) cosd(90) 0;
      0              0  1];

% servo body
A_top_s1 = [ 0 ; -6 ;  12];
B_top_s1 = [ 0 ;  6 ;  12];
C_top_s1 = [ 0 ;  6 ; -12];
D_top_s1 = [ 0 ; -6 ; -12];
top_s1   = R_z*[ A_top_s1 B_top_s1 C_top_s1 D_top_s1 ];
bottom_s1= top_s1   -[zeros(1,4);16.5*ones(1,4);zeros(1,4)] ;

top_s1   = top_s1   -[zeros(1,4);10.95*ones(1,4);5.85*ones(1,4)] ;
bottom_s1= bottom_s1-[zeros(1,4);10.95*ones(1,4);5.85*ones(1,4)] ;

top_s1   = top_s1   +[-21*ones(1,4);65*ones(1,4);17.85*ones(1,4)] ;
bottom_s1= bottom_s1+[-21*ones(1,4);65*ones(1,4);17.85*ones(1,4)] ;
% 
% R_z_angle =[cosd(angle) sind(angle) 0;
%            -sind(angle) cosd(angle) 0;
%             0              0        1];
A_top_s1 = top_s1(:,1);
B_top_s1 = top_s1(:,2);
C_top_s1 = top_s1(:,3);
D_top_s1 = top_s1(:,4);

A_bot_s1 = bottom_s1(:,1);
B_bot_s1 = bottom_s1(:,2);
C_bot_s1 = bottom_s1(:,3);
D_bot_s1 = bottom_s1(:,4);

face1_s1 =[A_top_s1 B_top_s1 B_bot_s1 A_bot_s1];
face2_s1 =[D_top_s1 C_top_s1 C_bot_s1 D_bot_s1];
face3_s1 =[A_top_s1 D_top_s1 D_bot_s1 A_bot_s1];
face4_s1 =[B_top_s1 C_top_s1 C_bot_s1 B_bot_s1];
face5_s1 =[A_top_s1 B_top_s1 C_top_s1 D_top_s1];
face6_s1 =[A_bot_s1 B_bot_s1 C_bot_s1 D_bot_s1];

fill3(face1_s1(1,:),face1_s1(2,:),face1_s1(3,:),red);%face1
fill3(face2_s1(1,:),face2_s1(2,:),face2_s1(3,:),red);%face2
fill3(face3_s1(1,:),face3_s1(2,:),face3_s1(3,:),red);%face3
fill3(face4_s1(1,:),face4_s1(2,:),face4_s1(3,:),red);%face4
fill3(face5_s1(1,:),face5_s1(2,:),face5_s1(3,:),red);%face5
fill3(face6_s1(1,:),face6_s1(2,:),face6_s1(3,:),red);%face6
%% Servo_s2
top_s2   = top_s1   +[42*ones(1,4);zeros(1,4);zeros(1,4)] ;
bottom_s2= bottom_s1+[42*ones(1,4);zeros(1,4);zeros(1,4)] ;

A_top_s2 = top_s2(:,1);
B_top_s2 = top_s2(:,2);
C_top_s2 = top_s2(:,3);
D_top_s2 = top_s2(:,4);

A_bot_s2 = bottom_s2(:,1);
B_bot_s2 = bottom_s2(:,2);
C_bot_s2 = bottom_s2(:,3);
D_bot_s2 = bottom_s2(:,4);

face1_s2 =[A_top_s2 B_top_s2 B_bot_s2 A_bot_s2];
face2_s2 =[D_top_s2 C_top_s2 C_bot_s2 D_bot_s2];
face3_s2 =[A_top_s2 D_top_s2 D_bot_s2 A_bot_s2];
face4_s2 =[B_top_s2 C_top_s2 C_bot_s2 B_bot_s2];
face5_s2 =[A_top_s2 B_top_s2 C_top_s2 D_top_s2];
face6_s2 =[A_bot_s2 B_bot_s2 C_bot_s2 D_bot_s2];

fill3(face1_s2(1,:),face1_s2(2,:),face1_s2(3,:),green);%face1
fill3(face2_s2(1,:),face2_s2(2,:),face2_s2(3,:),green);%face2
fill3(face3_s2(1,:),face3_s2(2,:),face3_s2(3,:),green);%face3
fill3(face4_s2(1,:),face4_s2(2,:),face4_s2(3,:),green);%face4
fill3(face5_s2(1,:),face5_s2(2,:),face5_s2(3,:),green);%face5
fill3(face6_s2(1,:),face6_s2(2,:),face6_s2(3,:),green);%face6

circle1_v_s2= circle1_v_s1+[42*ones(1,length(alph));zeros(1,length(alph));zeros(1,length(alph))];
circle2_v_s2= circle2_v_s1+[42*ones(1,length(alph));zeros(1,length(alph));zeros(1,length(alph))];

% pillar of servo2
fill3(circle1_v_s2(1,:),circle1_v_s2(2,:),circle1_v_s2(3,:),red);
fill3(circle2_v_s2(1,:),circle2_v_s2(2,:),circle2_v_s2(3,:),blue);
fill3([circle1_v_s2(1,:) fliplr(circle2_v_s2(1,:))],...
      [circle1_v_s2(2,:) fliplr(circle2_v_s2(2,:))],...
      [circle1_v_s2(3,:) fliplr(circle2_v_s2(3,:))],blue);
  
%% servo 3&4
R_z_120 =[cosd(120) sind(120) 0;
         -sind(120) cosd(120) 0;
          0              0    1];
 
circle1_v_s3=R_z_120*circle1_v_s1;
circle2_v_s3=R_z_120*circle2_v_s1;

circle1_v_s4=R_z_120*circle1_v_s2;
circle2_v_s4=R_z_120*circle2_v_s2;

top_s3   = R_z_120*top_s1;
bottom_s3= R_z_120*bottom_s1;

top_s4   = R_z_120*top_s2;
bottom_s4= R_z_120*bottom_s2;

% pillar of servo3
fill3(circle1_v_s3(1,:),circle1_v_s3(2,:),circle1_v_s3(3,:),red);
fill3(circle2_v_s3(1,:),circle2_v_s3(2,:),circle2_v_s3(3,:),blue);
fill3([circle1_v_s3(1,:) fliplr(circle2_v_s3(1,:))],...
      [circle1_v_s3(2,:) fliplr(circle2_v_s3(2,:))],...
      [circle1_v_s3(3,:) fliplr(circle2_v_s3(3,:))],blue);
% pillar of servo4
fill3(circle1_v_s4(1,:),circle1_v_s4(2,:),circle1_v_s4(3,:),red);
fill3(circle2_v_s4(1,:),circle2_v_s4(2,:),circle2_v_s4(3,:),blue);
fill3([circle1_v_s4(1,:) fliplr(circle2_v_s4(1,:))],...
      [circle1_v_s4(2,:) fliplr(circle2_v_s4(2,:))],...
      [circle1_v_s4(3,:) fliplr(circle2_v_s4(3,:))],blue);
%body of servo 3
A_top_s3 = top_s3(:,1);
B_top_s3 = top_s3(:,2);
C_top_s3 = top_s3(:,3);
D_top_s3 = top_s3(:,4);

A_bot_s3 = bottom_s3(:,1);
B_bot_s3 = bottom_s3(:,2);
C_bot_s3 = bottom_s3(:,3);
D_bot_s3 = bottom_s3(:,4);

face1_s3 =[A_top_s3 B_top_s3 B_bot_s3 A_bot_s3];
face2_s3 =[D_top_s3 C_top_s3 C_bot_s3 D_bot_s3];
face3_s3 =[A_top_s3 D_top_s3 D_bot_s3 A_bot_s3];
face4_s3 =[B_top_s3 C_top_s3 C_bot_s3 B_bot_s3];
face5_s3 =[A_top_s3 B_top_s3 C_top_s3 D_top_s3];
face6_s3 =[A_bot_s3 B_bot_s3 C_bot_s3 D_bot_s3];

fill3(face1_s3(1,:),face1_s3(2,:),face1_s3(3,:),green);%face1
fill3(face2_s3(1,:),face2_s3(2,:),face2_s3(3,:),green);%face2
fill3(face3_s3(1,:),face3_s3(2,:),face3_s3(3,:),green);%face3
fill3(face4_s3(1,:),face4_s3(2,:),face4_s3(3,:),green);%face4
fill3(face5_s3(1,:),face5_s3(2,:),face5_s3(3,:),green);%face5
fill3(face6_s3(1,:),face6_s3(2,:),face6_s3(3,:),green);%face6
%body of servo 4
A_top_s4 = top_s4(:,1);
B_top_s4 = top_s4(:,2);
C_top_s4 = top_s4(:,3);
D_top_s4 = top_s4(:,4);

A_bot_s4 = bottom_s4(:,1);
B_bot_s4 = bottom_s4(:,2);
C_bot_s4 = bottom_s4(:,3);
D_bot_s4 = bottom_s4(:,4);

face1_s4 =[A_top_s4 B_top_s4 B_bot_s4 A_bot_s4];
face2_s4 =[D_top_s4 C_top_s4 C_bot_s4 D_bot_s4];
face3_s4 =[A_top_s4 D_top_s4 D_bot_s4 A_bot_s4];
face4_s4 =[B_top_s4 C_top_s4 C_bot_s4 B_bot_s4];
face5_s4 =[A_top_s4 B_top_s4 C_top_s4 D_top_s4];
face6_s4 =[A_bot_s4 B_bot_s4 C_bot_s4 D_bot_s4];

fill3(face1_s4(1,:),face1_s4(2,:),face1_s4(3,:),green);%face1
fill3(face2_s4(1,:),face2_s4(2,:),face2_s4(3,:),green);%face2
fill3(face3_s4(1,:),face3_s4(2,:),face3_s4(3,:),green);%face3
fill3(face4_s4(1,:),face4_s4(2,:),face4_s4(3,:),green);%face4
fill3(face5_s4(1,:),face5_s4(2,:),face5_s4(3,:),green);%face5
fill3(face6_s4(1,:),face6_s4(2,:),face6_s4(3,:),green);%face6
%% servo 5&6
R_z_120 =[cosd(120) sind(120) 0;
         -sind(120) cosd(120) 0;
          0              0    1];
 
circle1_v_s5=R_z_120*circle1_v_s3;
circle2_v_s5=R_z_120*circle2_v_s3;

circle1_v_s6=R_z_120*circle1_v_s4;
circle2_v_s6=R_z_120*circle2_v_s4;

top_s5   = R_z_120*top_s3;
bottom_s5= R_z_120*bottom_s3;

top_s6   = R_z_120*top_s4;
bottom_s6= R_z_120*bottom_s4;

% pillar of servo5
fill3(circle1_v_s5(1,:),circle1_v_s5(2,:),circle1_v_s5(3,:),red);
fill3(circle2_v_s5(1,:),circle2_v_s5(2,:),circle2_v_s5(3,:),blue);
fill3([circle1_v_s5(1,:) fliplr(circle2_v_s5(1,:))],...
      [circle1_v_s5(2,:) fliplr(circle2_v_s5(2,:))],...
      [circle1_v_s5(3,:) fliplr(circle2_v_s5(3,:))],blue);
% pillar of servo6
fill3(circle1_v_s6(1,:),circle1_v_s6(2,:),circle1_v_s6(3,:),red);
fill3(circle2_v_s6(1,:),circle2_v_s6(2,:),circle2_v_s6(3,:),blue);
fill3([circle1_v_s6(1,:) fliplr(circle2_v_s6(1,:))],...
      [circle1_v_s6(2,:) fliplr(circle2_v_s6(2,:))],...
      [circle1_v_s6(3,:) fliplr(circle2_v_s6(3,:))],blue);
%body of servo 5
A_top_s5 = top_s5(:,1);
B_top_s5 = top_s5(:,2);
C_top_s5 = top_s5(:,3);
D_top_s5 = top_s5(:,4);

A_bot_s5 = bottom_s5(:,1);
B_bot_s5 = bottom_s5(:,2);
C_bot_s5 = bottom_s5(:,3);
D_bot_s5 = bottom_s5(:,4);

face1_s5 =[A_top_s5 B_top_s5 B_bot_s5 A_bot_s5];
face2_s5 =[D_top_s5 C_top_s5 C_bot_s5 D_bot_s5];
face3_s5 =[A_top_s5 D_top_s5 D_bot_s5 A_bot_s5];
face4_s5 =[B_top_s5 C_top_s5 C_bot_s5 B_bot_s5];
face5_s5 =[A_top_s5 B_top_s5 C_top_s5 D_top_s5];
face6_s5 =[A_bot_s5 B_bot_s5 C_bot_s5 D_bot_s5];

fill3(face1_s5(1,:),face1_s5(2,:),face1_s5(3,:),green);%face1
fill3(face2_s5(1,:),face2_s5(2,:),face2_s5(3,:),green);%face2
fill3(face3_s5(1,:),face3_s5(2,:),face3_s5(3,:),green);%face3
fill3(face4_s5(1,:),face4_s5(2,:),face4_s5(3,:),green);%face4
fill3(face5_s5(1,:),face5_s5(2,:),face5_s5(3,:),green);%face5
fill3(face6_s5(1,:),face6_s5(2,:),face6_s5(3,:),green);%face6
%body of servo 6
A_top_s6 = top_s6(:,1);
B_top_s6 = top_s6(:,2);
C_top_s6 = top_s6(:,3);
D_top_s6 = top_s6(:,4);

A_bot_s6 = bottom_s6(:,1);
B_bot_s6 = bottom_s6(:,2);
C_bot_s6 = bottom_s6(:,3);
D_bot_s6 = bottom_s6(:,4);

face1_s6 =[A_top_s6 B_top_s6 B_bot_s6 A_bot_s6];
face2_s6 =[D_top_s6 C_top_s6 C_bot_s6 D_bot_s6];
face3_s6 =[A_top_s6 D_top_s6 D_bot_s6 A_bot_s6];
face4_s6 =[B_top_s6 C_top_s6 C_bot_s6 B_bot_s6];
face5_s6 =[A_top_s6 B_top_s6 C_top_s6 D_top_s6];
face6_s6 =[A_bot_s6 B_bot_s6 C_bot_s6 D_bot_s6];

fill3(face1_s6(1,:),face1_s6(2,:),face1_s6(3,:),green);%face1
fill3(face2_s6(1,:),face2_s6(2,:),face2_s6(3,:),green);%face2
fill3(face3_s6(1,:),face3_s6(2,:),face3_s6(3,:),green);%face3
fill3(face4_s6(1,:),face4_s6(2,:),face4_s6(3,:),green);%face4
fill3(face5_s6(1,:),face5_s6(2,:),face5_s6(3,:),green);%face5
fill3(face6_s6(1,:),face6_s6(2,:),face6_s6(3,:),green);%face6

return