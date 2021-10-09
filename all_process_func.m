%%%%%%  ս����սȫ���̷��� %%%%%
function [tar_num,tar_ang,tar_dis,R_max,Pos_Aircraft,Pos_target,real_ang] = all_process_func(now_mode,now_pit,now_yaw,V_Aircraft,Pos_Aircraft,Pos_target,Phi,Sita,tar_find,tar_find_ang,tar_find_dis,tas_mode,tas_tar)
%%%%%%%%%%%%%%%%%%%%%%
%% output
%tar_num  ��ǰ�ҵ���Ŀ����;tar_ang ��ǰ�ҵ�Ŀ��Ƕȣ�tar_dis��ǰ�ҵ���Ŀ��Ķ�λ���룻
%R_max  ���ģ�����룻Pos_Aircraft �ɻ��ٶȣ� Pos_target Ŀ��λ�ã� real_ang Ŀ����ʵ�Ƕȣ�
%% input
%now_mode  �ĸ�ģʽ;now_pit now_yaw  �״ﲨ������Ƕȣ�V_Aircraft  �ɻ��ٶȣ�
%Pos_Aircraft  �ɻ�λ�ã�Pos_target  Ŀ��λ�ã�Phi,Sita  Ŀ����ʵ�Ƕȣ�tar_find ���ҵ���Ŀ������
%tar_find_ang ���ҵ���Ŀ��Ƕȣ�tar_find_dis ���ҵ���Ŀ�궨λ����
%%%%%%%%%%%%%%%%%%%%%%

clc
getd = @(p)path(path,p);
getd('fire_control_subpro/');
%% Ŀ��������
c = 3e8;
%   Ŀ�����꡾m��           X������              Y������                  Z������
V_target = [ [  1000;              2000;           2000] ,...
    [  -500;              1000;           -1000],...
    [  2000;              -3000;           1000]];
RCS_target=1*[1;1;1]; % �ٶ�Ŀ��RCS����������ڷ���ʱ����ں㶨
% Ŀ��ƽ������ɢ��������m^2��

%% Ŀ���Զʱ������TWS���ҵ�PRF���ϵ���Ƶ���߷��书�ʣ�����Ƶ��Ҳ���Խ���

if(now_mode == 1)
    %% �״�ϵͳ�������
    Pt= 5e7                     ;	% ���书�ʡ�W��
    Fc=10e6                     ;	% ����Ƶ�ʡ�Hz��
    B=1e5                       ;	% ����Hz��
    Fs=0.1*Fc                   ;	% �����ʡ�Hz��
    Tr=2e-3    ;%(500Hz)                 ;	% �����ظ����ڡ��롿
    Tp=Tr/2                     ;	% �����ȡ�΢�롿
    [tar_find,tar_find_ang,tar_find_dis,tar_find_v,tar_real_posi,radar_real_posi,real_ang] = gui_Tws_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita,now_pit,now_yaw,tar_find,tar_find_ang,tar_find_dis);
   
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    tar_num = tar_find;
    tar_dis = tar_find_dis;
    tar_ang = tar_find_ang;
    R_max = c*Tr/2;
elseif(now_mode == 2)
    %% Ŀ��ӽ�ʱ����TAS����PRF������Ƶ�����ͷ��书�ʣ���߲�����
    Pt= 5e6                     ;	% ���书�ʡ�W��
    Fc=10e8                     ;	% ����Ƶ�ʡ�Hz��
    B=1e7                       ;	% ����Hz��
    Fs=0.1*Fc                   ;	% �����ʡ�Hz��
    Tr=2e-4        ;% (5000Hz)            ;	% �����ظ����ڡ��롿
    Tp=Tr/2                     ;	% �����ȡ�΢�롿
    
    [tar_find,tar_find_ang,tar_find_dis,tar_find_v,tar_real_posi,radar_real_posi,real_ang] = gui_Tas_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita,now_pit,now_yaw,tar_find,tar_find_ang,tar_find_dis,tas_mode,tas_tar);

    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    tar_num = tar_find;
    tar_dis = tar_find_dis;
    tar_ang = tar_find_ang;
    R_max = c*Tr/2;
elseif(now_mode == 3)
    %% Ŀ���൱��ʱ����STTģʽ������׼�����Ŀ�꣬��PRF������Ƶ�����ͷ��书��
    Pt= 5e5                     ;	% ���书�ʡ�W��
    Fc=10e8                     ;	% ����Ƶ�ʡ�Hz��
    B=1e8                       ;	% ����Hz��
    Fs=Fc                   ;	% �����ʡ�Hz��
    Tr=2e-5                     ;	% �����ظ����ڡ��롿
    Tp=Tr/2                     ;	% �����ȡ�΢�롿

    [tar_find,tar_find_ang,tar_find_dis,tar_find_v,tar_real_posi,radar_real_posi,real_ang] = gui_Tas_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita,now_pit,now_yaw,tar_find,tar_find_ang,tar_find_dis,tas_mode,tas_tar);
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    tar_ang = tar_find_ang;
    R_max = c*Tr/2;
    tar_num = tar_find;
    tar_dis = tar_find_dis;
end

