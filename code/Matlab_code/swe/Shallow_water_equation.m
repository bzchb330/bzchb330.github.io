%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Applied High Performance Computing MCEN90031
%
% Assignment1:	Shallow water equation
%
% Problem:	dvx/dt+vx*(dvx/dx+dvx/dy)=-g*dh/dx
%           dvy/dt+vy*(dvy/dx+dvy/dy)=-g*dh/dy
%           dh/dt+d(vxh)/dx+d(vyh/dy)=0
%
% Method:	Finite Difference Method with Sixth order central differences
%			and Fourth Order Runge-Kutta Method
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Shallow_water_equation()

% dt=0.05;
% 
% video_file = VideoWriter('SWE','MPEG-4');
% video_file.FrameRate = 1/dt;
% open(video_file);

%% Sixth Order Central Difference Method Stability Analysis
line1=[1 1 1 1 1 1 1];
line2=[-3 -2 -1 0 1 2 3];
line3=[4.5 2 0.5 0 0.5 2 4.5];
line4=[-27/6 -4/3 -1/6 0 1/6 4/3 27/6];
line5=[81/24 16/24 1/24 0 1/24 16/24 81/24];
line6=[-243/120 -32/120 -1/120 0 1/120 32/120 243/120];
line7=[729/720 64/720 1/720 0 1/720 64/720 729/720];
[V D]=eig([line1;line2;line3;line4;line5;line6;line7]);
Delta_t             =0.2;
[X, Y]              = meshgrid(-4:0.1:4, -4:0.1:4);
Z                   = X + j*Y;
sigma4              = abs(1 + Z + (Z.^2)/2 + (Z.^3)/6 + (Z.^4)/24);
figure('WindowStyle', 'docked');
contourf(X, Y, sigma4, [1 1],'b','linewidth',2);
hold on
plot(real(diag(D)*Delta_t),imag(diag(D))*Delta_t, '.', 'MarkerSize', 20);
xlabel('\lambda_{Re}\Delta t');
ylabel('\lambda_{Im}\Delta t');
grid on
%% intial conditions
x_min               = 0.00;
x_max               = 100.00;
y_min               = 0.00;
y_max               = 100.00;
t_min               =0.0;
t_max               =100;
Delta_x             = 1;
Delta_y             = 1;
N_x                 = length(x_min:Delta_x:y_max);
N_y                 = length(y_min:Delta_y:y_max);
N_t                 =(t_max-t_min)/Delta_t;
g                   = 9.81;
[x, y]              = meshgrid(x_min:Delta_x:x_max, y_min:Delta_y:y_max);

h(:,:)              =1+0.5*exp((-1/25)*((x-30).^2 + (y-30).^2));
vx                  =zeros(N_x,N_y);
vy                  =zeros(N_x,N_y);
phi(:,:,1)          =h;
phi(:,:,2)          =vx;
phi(:,:,3)          =vy;
a                   =1/6;
b                   =1/3;

figure('WindowStyle', 'docked');
solution=surf(x,y,phi(:,:,1));
axis([0 100 0 100 0 2]);
xlabel('v_x'); 
ylabel('v_y');
zlabel('h');
alpha(0.8);
view(60,20);
colormap('Winter');
shading interp
daspect([1 1 0.05]);
%% time marching loop
t=t_min;
for i=1:N_t-1
    k1=RK4(phi,N_x,N_y,g,Delta_x,Delta_y);
    k2=RK4(phi+0.5*Delta_t*k1,N_x,N_y,g,Delta_x,Delta_y);
    k3=RK4(phi+0.5*Delta_t*k2,N_x,N_y,g,Delta_x,Delta_y);
    k4=RK4(phi+Delta_t*k3,N_x,N_y,g,Delta_x,Delta_y);
    phi=phi+Delta_t*(a*k1+b*k2+b*k3+a*k4);

    set(solution,'Zdata',phi(:,:,1));
    title(['t = ' num2str(t),' s' ]);
    t=t+Delta_t;
    drawnow;
%     frame = getframe;
%     writeVideo(video_file,frame);
end
%  close(video_file)
return

function k=RK4(phi,N_x,N_y,g,Delta_x,Delta_y)

k=zeros(N_x,N_y,3);   

for i=1:N_x
        ip1=[i-3 i-2 i-1 i+1 i+2 i+3];
        if(i==1)
         ip1=[N_x-2 N_x-1 N_x 2 3 4];
        elseif(i==2)
         ip1=[N_x-1 N_x 1 3 4 5];
        elseif(i==3)
         ip1=[N_x 1 2 4 5 6];
        elseif(i==N_x)
         ip1=[N_x-3 N_x-2 N_x-1 1 2 3];
        elseif(i==N_x-1)
         ip1=[N_x-4 N_x-3 N_x-2 N_x 1 2];
        elseif(i==N_x-2)
         ip1=[N_x-5 N_x-4 N_x-3 N_x-1 N_x 1];
        end
    for j=1:N_y
         jp2=[j-3 j-2 j-1 j+1 j+2 j+3];
        if(j==1)
         jp2=[N_y-2 N_y-1 N_y 2 3 4];
        elseif(j==2)
         jp2=[N_y-1 N_y 1 3 4 5];
        elseif(j==3)
         jp2=[N_y 1 2 4 5 6];
        elseif(j==N_y)
         jp2=[N_y-3 N_y-2 N_y-1 1 2 3];
        elseif(j==N_y-1)
         jp2=[N_y-4 N_y-3 N_y-2 N_y 1 2];
        elseif(j==N_y-2)
         jp2=[N_y-5 N_y-4 N_y-3 N_y-1 N_y 1];
        end
           
k(i,j,2)=-phi(i,j,2)/Delta_x*(-1/60*phi(ip1(1),j,2)+3/20*phi(ip1(2),j,2)+(-3/4)*phi(ip1(3),j,2)+3/4*phi(ip1(4),j,2)+(-3/20)*phi(ip1(5),j,2)+1/60*phi(ip1(6),j,2))...
         -phi(i,j,3)/Delta_y*(-1/60*phi(i,jp2(1),2)+3/20*phi(i,jp2(2),2)+(-3/4)*phi(i,jp2(3),2)+3/4*phi(i,jp2(4),2)+(-3/20)*phi(i,jp2(5),2)+1/60*phi(i,jp2(6),2))...
                  -g/Delta_x*(-1/60*phi(ip1(1),j,1)+3/20*phi(ip1(2),j,1)+(-3/4)*phi(ip1(3),j,1)+3/4*phi(ip1(4),j,1)+(-3/20)*phi(ip1(5),j,1)+1/60*phi(ip1(6),j,1));

k(i,j,3)=-phi(i,j,2)/Delta_x*(-1/60*phi(ip1(1),j,3)+3/20*phi(ip1(2),j,3)+(-3/4)*phi(ip1(3),j,3)+3/4*phi(ip1(4),j,3)+(-3/20)*phi(ip1(5),j,3)+1/60*phi(ip1(6),j,3))...
         -phi(i,j,3)/Delta_y*(-1/60*phi(i,jp2(1),3)+3/20*phi(i,jp2(2),3)+(-3/4)*phi(i,jp2(3),3)+3/4*phi(i,jp2(4),3)+(-3/20)*phi(i,jp2(5),3)+1/60*phi(i,jp2(6),3))...
                  -g/Delta_y*(-1/60*phi(i,jp2(1),1)+3/20*phi(i,jp2(2),1)+(-3/4)*phi(i,jp2(3),1)+3/4*phi(i,jp2(4),1)+(-3/20)*phi(i,jp2(5),1)+1/60*phi(i,jp2(6),1));

k(i,j,1)=-1/Delta_x*(-1/60*phi(ip1(1),j,1)*phi(ip1(1),j,2)+3/20*phi(ip1(2),j,1)*phi(ip1(2),j,2)+(-3/4)*phi(ip1(3),j,1)*phi(ip1(3),j,2)+3/4*phi(ip1(4),j,1)*phi(ip1(4),j,2)+(-3/20)*phi(ip1(5),j,1)*phi(ip1(5),j,2)+1/60*phi(ip1(6),j,1)*phi(ip1(6),j,2))...
         -1/Delta_y*(-1/60*phi(i,jp2(1),1)*phi(i,jp2(1),3)+3/20*phi(i,jp2(2),1)*phi(i,jp2(2),3)+(-3/4)*phi(i,jp2(3),1)*phi(i,jp2(3),3)+3/4*phi(i,jp2(4),1)*phi(i,jp2(4),3)+(-3/20)*phi(i,jp2(5),1)*phi(i,jp2(5),3)+1/60*phi(i,jp2(6),1)*phi(i,jp2(6),3));

    end
end
return
