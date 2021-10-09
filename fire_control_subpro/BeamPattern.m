function [beam,a1,p1] = BeamPattern(N_azi,N_pit,lamada,azi_ang,pit_ang)
%   BeamPattern：波束生成程序，根据预设参数，进行相控阵波束仿真。
%   Input:  N_azi: 方位向天线个数
%           N_pit: 俯仰向天线个数
%           lamada：波长
%           azi_ang: 方位向波束指向  °
%           pit_ang: 俯仰向波束指向  °
%   Output：输出波束图
%   默认计算-70°至70°的波束图，角度间隔0.5°，需要精确角度需要进行插值
%   By Sun Xiping @ SYSU 2020-08-24
%% 平面阵列天线方向图
% 1i=sqrt(-1);
N1=N_azi;   % 天线阵元数
N2=N_pit;
lambda=lamada; % 载波信号波长:10cm

% 目标信号方向
dir1_sig=azi_ang*pi/180;  % 期望信号方位角
dir2_sig=pit_ang*pi/180;  % 期望信号俯仰角
dx=lambda/2; % 行阵元间距
dy=lambda/2; % 列阵元间距

theta1=sin(dir1_sig)*cos(dir2_sig);
theta2=sin(dir2_sig);

X=(0:(N1-1))*dx;
Y=(0:(N2-1))*dy;
% X2=kron(ones(1,N2),X);
% Y2=kron(Y,ones(1,N1));
% figure
% plot(X2,Y2,'.');axis equal;grid on;
% title('天线阵');xlabel('距离（m）');ylabel('距离（m）');

steer_sig1=exp(1i*(2*pi/lambda*X*theta1)); % 主天线信号的导向矢量  
steer_sig3=exp(1i*(2*pi/lambda*Y*theta2));
steer_sig2=kron(steer_sig1,steer_sig3);                                      %???????????为啥这样乘
normal_W=steer_sig2;

% Forming beam pattern 
win_n1 = hamming(N1).';
win_n2 = hamming(N2).';
k1=1;
for thta=-20:0.5:20
    k2=1;
    for phi=-20:0.5:20
        th_a1=win_n1.*exp(1i*2*pi/lambda*X*sin(thta*pi/180)*cos(phi*pi/180));
        th_p1=win_n2.*exp(1i*2*pi/lambda*Y*sin(phi*pi/180));
        th_all1=kron(th_a1,th_p1).';                                        %?????????????????
        yy(k1,k2)=abs(((normal_W.')')*th_all1);
        k2=k2+1;
    end
    k1=k1+1;
end
beam = yy;
p1=-20:0.5:20;
a1=-20:0.5:20;          %天线方向图的方向范围
