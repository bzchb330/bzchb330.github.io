dt=0.01;
% video_file = VideoWriter('sp_phi','MPEG-4');
% video_file.FrameRate = 1/dt;
% open(video_file);

x_t=-20:0.5:20;
y_t=0;
z_t=95;
psi=0;
theta=0;
phi=0;

for i=1:length(x_t)
    stewart_platform(x_t(i),y_t,z_t,psi,theta,phi)
    set(gcf,'position',[50,200,800,600]);
%     frame = getframe;
%     writeVideo(video_file,frame);
    pause(dt);
end
% close(video_file)
