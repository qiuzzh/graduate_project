%%%%%%  战机作战全过程仿真 %%%%%

clear;
close all
clc
getd = @(p)path(path,p);
getd('fire_control_subpro/');
%% 载机平台参数
Pos_Aircraft=[0;0;0]        ;   % 载机三维坐标【m】
V_Aircraft=[1000;3000;2000]    ;	% 载机速度矢量【m/s】
%% 雷达系统仿真参数
Pt= 5e7                     ;	% 发射功率【W】
Fc=10e6                     ;	% 中心频率【Hz】
B=1e5                       ;	% 带宽【Hz】
Fs=0.1*Fc                   ;	% 采样率【Hz】
Tr=2e-3                     ;	% 脉冲重复周期【秒】 
Tp=Tr/2                     ;	% 脉冲宽度【微秒】

%% 目标仿真参数
% 起始目标角度  这里的角度是相对飞机而言的
Phi = [8 3 -5 ]./180*pi;                 %俯仰
Sita = [-20 -1 15 ]./180*pi;             %方位
%%%%
Pos_target=[ Pos_Aircraft+ 5e4*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))] ,...
    Pos_Aircraft+ 3e4*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...
    Pos_Aircraft+6e4*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))]];                   % 目标4
%   目标坐标【m】           X轴坐标              Y轴坐标                  Z轴坐标
V_target = [ [  1000;              2000;           2000] ,...
   [  -500;              1000;           -1000],...
   [  2000;              -3000;           1000]];    
RCS_target=1*[1;1;1]; % 假定目标RCS慢起伏，即在仿真时间段内恒定
% 目标平均后向散射截面积【m^2】

%% 目标较远时，先用TWS，且低PRF，较低载频，高发射功率，采样频率也可以降低降低
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
    V = 10000;            %飞机径向速度（m/s）
    V_Aircraft = V*[cos(tar_find_ang(1,min_index))*sin(tar_find_ang(2,min_index)); cos(tar_find_ang(1,min_index))*cos(tar_find_ang(2,min_index)); sin(tar_find_ang(1,min_index))];    %朝最近的目标飞去
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    Phi = real_ang(2,:);
    Sita = real_ang(1,:);
    if(dis_min<1e4)
        break
    end
end


%% 目标接近时，用TAS，中PRF，高载频，降低发射功率，提高采样率
Pt= 5e6                     ;	% 发射功率【W】
Fc=10e8                     ;	% 中心频率【Hz】
B=1e7                       ;	% 带宽【Hz】
Fs=0.1*Fc                   ;	% 采样率【Hz】
Tr=2e-4                     ;	% 脉冲重复周期【秒】
Tp=Tr/2                     ;	% 脉冲宽度【微秒】

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
    V = 10000;            %飞机径向速度（m/s）
    V_Aircraft = V*[cos(tar_find_ang(1,min_index))*sin(tar_find_ang(2,min_index)); cos(tar_find_ang(1,min_index))*cos(tar_find_ang(2,min_index)); sin(tar_find_ang(1,min_index))];    %朝最近的目标飞去
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    Phi = real_ang(2,:);
    Sita = real_ang(1,:);
    if(dis_min<3e3)
        break
    end
end

%% 目标相当近时，用STT模式，并瞄准最近的目标，高PRF，高载频，降低发射功率
Pt= 5e5                     ;	% 发射功率【W】
Fc=10e8                     ;	% 中心频率【Hz】
B=1e8                       ;	% 带宽【Hz】
Fs=Fc                   ;	% 采样率【Hz】
Tr=2e-5                     ;	% 脉冲重复周期【秒】
Tp=Tr/2                     ;	% 脉冲宽度【微秒】         
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
    V = 10000;            %飞机径向速度（m/s）
    V_Aircraft = V*[cos(tar_find_ang(1,min_index))*sin(tar_find_ang(2,min_index)); cos(tar_find_ang(1,min_index))*cos(tar_find_ang(2,min_index)); sin(tar_find_ang(1,min_index))];    %朝最近的目标飞去
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    Phi = real_ang(2,:);
    Sita = real_ang(1,:);
    if(dis_min<1e2)
        break
    end
end
