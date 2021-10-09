%%%%%%  战机作战全过程仿真 %%%%%
function [tar_num,tar_ang,tar_dis,R_max,Pos_Aircraft,Pos_target,real_ang] = all_process_func(now_mode,now_pit,now_yaw,V_Aircraft,Pos_Aircraft,Pos_target,Phi,Sita,tar_find,tar_find_ang,tar_find_dis,tas_mode,tas_tar)
%%%%%%%%%%%%%%%%%%%%%%
%% output
%tar_num  当前找到的目标数;tar_ang 当前找的目标角度；tar_dis当前找到的目标的定位距离；
%R_max  最大不模糊距离；Pos_Aircraft 飞机速度； Pos_target 目标位置； real_ang 目标真实角度；
%% input
%now_mode  哪个模式;now_pit now_yaw  雷达波束照射角度；V_Aircraft  飞机速度；
%Pos_Aircraft  飞机位置；Pos_target  目标位置；Phi,Sita  目标真实角度；tar_find 已找到的目标数；
%tar_find_ang 已找到的目标角度；tar_find_dis 已找到的目标定位距离
%%%%%%%%%%%%%%%%%%%%%%

clc
getd = @(p)path(path,p);
getd('fire_control_subpro/');
%% 目标仿真参数
c = 3e8;
%   目标坐标【m】           X轴坐标              Y轴坐标                  Z轴坐标
V_target = [ [  1000;              2000;           2000] ,...
    [  -500;              1000;           -1000],...
    [  2000;              -3000;           1000]];
RCS_target=1*[1;1;1]; % 假定目标RCS慢起伏，即在仿真时间段内恒定
% 目标平均后向散射截面积【m^2】

%% 目标较远时，先用TWS，且低PRF，较低载频，高发射功率，采样频率也可以降低

if(now_mode == 1)
    %% 雷达系统仿真参数
    Pt= 5e7                     ;	% 发射功率【W】
    Fc=10e6                     ;	% 中心频率【Hz】
    B=1e5                       ;	% 带宽【Hz】
    Fs=0.1*Fc                   ;	% 采样率【Hz】
    Tr=2e-3    ;%(500Hz)                 ;	% 脉冲重复周期【秒】
    Tp=Tr/2                     ;	% 脉冲宽度【微秒】
    [tar_find,tar_find_ang,tar_find_dis,tar_find_v,tar_real_posi,radar_real_posi,real_ang] = gui_Tws_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita,now_pit,now_yaw,tar_find,tar_find_ang,tar_find_dis);
   
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    tar_num = tar_find;
    tar_dis = tar_find_dis;
    tar_ang = tar_find_ang;
    R_max = c*Tr/2;
elseif(now_mode == 2)
    %% 目标接近时，用TAS，中PRF，高载频，降低发射功率，提高采样率
    Pt= 5e6                     ;	% 发射功率【W】
    Fc=10e8                     ;	% 中心频率【Hz】
    B=1e7                       ;	% 带宽【Hz】
    Fs=0.1*Fc                   ;	% 采样率【Hz】
    Tr=2e-4        ;% (5000Hz)            ;	% 脉冲重复周期【秒】
    Tp=Tr/2                     ;	% 脉冲宽度【微秒】
    
    [tar_find,tar_find_ang,tar_find_dis,tar_find_v,tar_real_posi,radar_real_posi,real_ang] = gui_Tas_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita,now_pit,now_yaw,tar_find,tar_find_ang,tar_find_dis,tas_mode,tas_tar);

    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    tar_num = tar_find;
    tar_dis = tar_find_dis;
    tar_ang = tar_find_ang;
    R_max = c*Tr/2;
elseif(now_mode == 3)
    %% 目标相当近时，用STT模式，并瞄准最近的目标，高PRF，高载频，降低发射功率
    Pt= 5e5                     ;	% 发射功率【W】
    Fc=10e8                     ;	% 中心频率【Hz】
    B=1e8                       ;	% 带宽【Hz】
    Fs=Fc                   ;	% 采样率【Hz】
    Tr=2e-5                     ;	% 脉冲重复周期【秒】
    Tp=Tr/2                     ;	% 脉冲宽度【微秒】

    [tar_find,tar_find_ang,tar_find_dis,tar_find_v,tar_real_posi,radar_real_posi,real_ang] = gui_Tas_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita,now_pit,now_yaw,tar_find,tar_find_ang,tar_find_dis,tas_mode,tas_tar);
    Pos_target = tar_real_posi;
    Pos_Aircraft = radar_real_posi;
    tar_ang = tar_find_ang;
    R_max = c*Tr/2;
    tar_num = tar_find;
    tar_dis = tar_find_dis;
end

