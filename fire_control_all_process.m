%%%%%%  ս����սȫ���̷��� %%%%%

clear;
close all
clc
getd = @(p)path(path,p);
getd('fire_control_subpro/');
%% �ػ�ƽ̨����
Pos_Aircraft=[0;0;0]        ;   % �ػ���ά���꡾m��
V_Aircraft=[1000;3000;2000]    ;	% �ػ��ٶ�ʸ����m/s��
%% �״�ϵͳ�������
Pt= 5e7                     ;	% ���书�ʡ�W��
Fc=10e6                     ;	% ����Ƶ�ʡ�Hz��
B=1e5                       ;	% ����Hz��
Fs=0.1*Fc                   ;	% �����ʡ�Hz��
Tr=2e-3                     ;	% �����ظ����ڡ��롿 
Tp=Tr/2                     ;	% �����ȡ�΢�롿

%% Ŀ��������
% ��ʼĿ��Ƕ�  ����ĽǶ�����Էɻ����Ե�
Phi = [8 3 -5 ]./180*pi;                 %����
Sita = [-20 -1 15 ]./180*pi;             %��λ
%%%%
Pos_target=[ Pos_Aircraft+ 5e4*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))] ,...
    Pos_Aircraft+ 3e4*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...
    Pos_Aircraft+6e4*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))]];                   % Ŀ��4
%   Ŀ�����꡾m��           X������              Y������                  Z������
V_target = [ [  1000;              2000;           2000] ,...
   [  -500;              1000;           -1000],...
   [  2000;              -3000;           1000]];    
RCS_target=1*[1;1;1]; % �ٶ�Ŀ��RCS����������ڷ���ʱ����ں㶨
% Ŀ��ƽ������ɢ��������m^2��

%% Ŀ���Զʱ������TWS���ҵ�PRF���ϵ���Ƶ���߷��书�ʣ�����Ƶ��Ҳ���Խ��ͽ���
while(1)
    [tar_find,tar_find_ang,tar_find_dis,tar_find_v,tar_real_posi,radar_real_posi,real_ang] = Tws_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita);
    dis_min = 1e10;
    min_index = 0;
    for kk = 1:tar_find
        if(tar_find_dis(1,kk) < dis_min)
            dis_min = tar_find_dis(1,kk);
            min_index = kk;
        end
    end
    V = 10000;            %�ɻ������ٶȣ�m/s��
    V_Aircraft = V*[cos(tar_find_ang(1,min_index))*sin(tar_find_ang(2,min_index)); cos(tar_find_ang(1,min_index))*cos(tar_find_ang(2,min_index)); sin(tar_find_ang(1,min_index))];    %�������Ŀ���ȥ
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    Phi = real_ang(2,:);
    Sita = real_ang(1,:);
    if(dis_min<1e4)
        break
    end
end


%% Ŀ��ӽ�ʱ����TAS����PRF������Ƶ�����ͷ��书�ʣ���߲�����
Pt= 5e6                     ;	% ���书�ʡ�W��
Fc=10e8                     ;	% ����Ƶ�ʡ�Hz��
B=1e7                       ;	% ����Hz��
Fs=0.1*Fc                   ;	% �����ʡ�Hz��
Tr=2e-4                     ;	% �����ظ����ڡ��롿
Tp=Tr/2                     ;	% �����ȡ�΢�롿

while(1)
    [tar_find,tar_find_ang,tar_find_dis,tar_find_v,tar_real_posi,radar_real_posi,real_ang] = Tas_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita);
    dis_min = 1e10;
    min_index = 0;
    for kk = 1:tar_find
        if(tar_find_dis(1,kk) < dis_min)
            dis_min = tar_find_dis(1,kk);
            min_index = kk;
        end
    end
    V = 10000;            %�ɻ������ٶȣ�m/s��
    V_Aircraft = V*[cos(tar_find_ang(1,min_index))*sin(tar_find_ang(2,min_index)); cos(tar_find_ang(1,min_index))*cos(tar_find_ang(2,min_index)); sin(tar_find_ang(1,min_index))];    %�������Ŀ���ȥ
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    Phi = real_ang(2,:);
    Sita = real_ang(1,:);
    if(dis_min<3e3)
        break
    end
end

%% Ŀ���൱��ʱ����STTģʽ������׼�����Ŀ�꣬��PRF������Ƶ�����ͷ��书��
Pt= 5e5                     ;	% ���书�ʡ�W��
Fc=10e8                     ;	% ����Ƶ�ʡ�Hz��
B=1e8                       ;	% ����Hz��
Fs=Fc                   ;	% �����ʡ�Hz��
Tr=2e-5                     ;	% �����ظ����ڡ��롿
Tp=Tr/2                     ;	% �����ȡ�΢�롿         
tar_rock = 0;
%tar_find_ang = [Phi(2)*180/pi;Sita(2)*180/pi];
%min_index = 1;
while(1)
    tar_rock = 1;
    tar_ang = tar_find_ang(:,min_index);
    [tar_find_ang,tar_find_dis,tar_find_v,tar_real_posi,radar_real_posi,real_ang,tar_rock,vd] = Stt_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita,tar_rock,tar_ang);
    dis_min = 1e10;
    min_index = 0;
    for kk = 1:1
        if(tar_find_dis(1,kk) < dis_min)
            dis_min = tar_find_dis(1,kk);
            min_index = kk;
        end
    end
    V = 10000;            %�ɻ������ٶȣ�m/s��
    V_Aircraft = V*[cos(tar_find_ang(1,min_index))*sin(tar_find_ang(2,min_index)); cos(tar_find_ang(1,min_index))*cos(tar_find_ang(2,min_index)); sin(tar_find_ang(1,min_index))];    %�������Ŀ���ȥ
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    Phi = real_ang(2,:);
    Sita = real_ang(1,:);
    if(dis_min<1e2)
        break
    end
end
