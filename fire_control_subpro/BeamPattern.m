function [beam,a1,p1] = BeamPattern(N_azi,N_pit,lamada,azi_ang,pit_ang)
%   BeamPattern���������ɳ��򣬸���Ԥ���������������������档
%   Input:  N_azi: ��λ�����߸���
%           N_pit: ���������߸���
%           lamada������
%           azi_ang: ��λ����ָ��  ��
%           pit_ang: ��������ָ��  ��
%   Output���������ͼ
%   Ĭ�ϼ���-70����70��Ĳ���ͼ���Ƕȼ��0.5�㣬��Ҫ��ȷ�Ƕ���Ҫ���в�ֵ
%   By Sun Xiping @ SYSU 2020-08-24
%% ƽ���������߷���ͼ
% 1i=sqrt(-1);
N1=N_azi;   % ������Ԫ��
N2=N_pit;
lambda=lamada; % �ز��źŲ���:10cm

% Ŀ���źŷ���
dir1_sig=azi_ang*pi/180;  % �����źŷ�λ��
dir2_sig=pit_ang*pi/180;  % �����źŸ�����
dx=lambda/2; % ����Ԫ���
dy=lambda/2; % ����Ԫ���

theta1=sin(dir1_sig)*cos(dir2_sig);
theta2=sin(dir2_sig);

X=(0:(N1-1))*dx;
Y=(0:(N2-1))*dy;
% X2=kron(ones(1,N2),X);
% Y2=kron(Y,ones(1,N1));
% figure
% plot(X2,Y2,'.');axis equal;grid on;
% title('������');xlabel('���루m��');ylabel('���루m��');

steer_sig1=exp(1i*(2*pi/lambda*X*theta1)); % �������źŵĵ���ʸ��  
steer_sig3=exp(1i*(2*pi/lambda*Y*theta2));
steer_sig2=kron(steer_sig1,steer_sig3);                                      %???????????Ϊɶ������
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
a1=-20:0.5:20;          %���߷���ͼ�ķ���Χ
