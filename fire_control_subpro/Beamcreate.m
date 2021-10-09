function [beam,a1,p1] = Beamcreate(N_azi,N_pit,lamada,azi_ang,pit_ang,ane_ang)
%N_azi;N_pit 天线阵元数
%lambda  波长
%azi_ang 此时方位角
%pit_ang 此时俯仰角
%ane_ang 天线波束范围

%% 平面阵列天线方向图
% 1i=sqrt(-1);
N1=N_azi;   % 天线阵元数
N2=N_pit;
lambda=lamada; % 载波信号波长:10cm

% 目标信号方向
dir1_sig=azi_ang*pi/180;  % 期望信号方位角
dir2_sig=pit_ang*pi/180;  % 期望信号俯仰角
ang_yaw = azi_ang - ane_ang/2:0.5:azi_ang + ane_ang/2;
ang_pit = pit_ang - ane_ang/2:0.5:pit_ang + ane_ang/2;
dir_yaw = ang_yaw*pi/180;
dir_pit = ang_pit*pi/180;
dx=lambda/4; % 行阵元间距
dy=lambda/4; % 列阵元间距

X=(0:(N1-1))*dx;
Y=(0:(N2-1))*dy; 
theta1=sin(dir1_sig)*cos(dir2_sig);
theta2=sin(dir2_sig);
for k1 = 1:length(dir_yaw)
    for k2 = 1:length(dir_pit)
        sig = exp(1i*(2*pi/lambda*X'*sin(dir_yaw(k1))*cos(dir_pit(k2)) - 2*pi/lambda*X'*theta1))*exp(1i*(2*pi/lambda*Y*sin(dir_pit(k2)) - 2*pi/lambda*Y*theta2));
        y(k1,k2) = abs((sum(sum(sig))));
    end
end
beam = y;
a1 = -ane_ang/2:0.5:ane_ang/2;
p1 = -ane_ang/2:0.5:ane_ang/2;
    %figure;imagesc(a1,p1,beam)
    %xlabel('俯仰角°'); ylabel('水平角°'); 
