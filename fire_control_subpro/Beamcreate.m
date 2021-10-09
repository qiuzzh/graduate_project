function [beam,a1,p1] = Beamcreate(N_azi,N_pit,lamada,azi_ang,pit_ang,ane_ang)
%N_azi;N_pit ������Ԫ��
%lambda  ����
%azi_ang ��ʱ��λ��
%pit_ang ��ʱ������
%ane_ang ���߲�����Χ

%% ƽ���������߷���ͼ
% 1i=sqrt(-1);
N1=N_azi;   % ������Ԫ��
N2=N_pit;
lambda=lamada; % �ز��źŲ���:10cm

% Ŀ���źŷ���
dir1_sig=azi_ang*pi/180;  % �����źŷ�λ��
dir2_sig=pit_ang*pi/180;  % �����źŸ�����
ang_yaw = azi_ang - ane_ang/2:0.5:azi_ang + ane_ang/2;
ang_pit = pit_ang - ane_ang/2:0.5:pit_ang + ane_ang/2;
dir_yaw = ang_yaw*pi/180;
dir_pit = ang_pit*pi/180;
dx=lambda/4; % ����Ԫ���
dy=lambda/4; % ����Ԫ���

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
    %xlabel('�����ǡ�'); ylabel('ˮƽ�ǡ�'); 
