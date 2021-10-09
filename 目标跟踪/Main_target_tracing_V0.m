%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       简单目标跟踪
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc


%% 基本参数
Tr = 10e-2   ;

%% 跟踪器设计
range_g = 1 ;   % 跟踪距离门

%% 目标航迹设定
tar_num = 5        ;   % 集群目标个数
tar_v = [3 0 -10]*5          ;   % 集群目标整体速度，每个目标再进行速度分离，以获得多普勒分辨
tar_start_range = [-1000 1000 5000];   % 目标起始距离
swarm_r = [(1:tar_num)*5;       %三维位置
    zeros(1,tar_num);
    zeros(1,tar_num)];

% swarm_v_f = [rand(1,tar_num)*10+10;
%             rand(1,tar_num)*10+10;
%             rand(1,tar_num)*10+10;];
% swarm_v_f = swarm_v_f * 0.001;
% save swarm_v_f
load swarm_v_f
% swarm_v_a = [rand(1,tar_num)*10];
% save swarm_v_a 
load swarm_v_a
% star_t = rand(1)*100;
% save star_t
load star_t
nan = 1024;     %周期数
swarm_ran = zeros(3,tar_num,nan);   %3维*目标*时间
for ki = 1:nan
    t = star_t+ki*Tr*1;
    v_r = tar_v*t;
    swarm_ran(:,:,ki) = swarm_r + swarm_v_a.*sin(2*pi*swarm_v_f*t) + v_r.';     %位置
end



figure; hold on
for ki = 1:tar_num
    plot3(squeeze(swarm_ran(1,ki,:)),squeeze(swarm_ran(2,ki,:)),...
        squeeze(swarm_ran(3,ki,:)),'.','color',[rand rand rand]);
end
xlabel('方位/m');ylabel('俯仰/m');zlabel('距离/m');
title('集群目标三维运动航迹')


% 加虚警
false_a = zeros(3,4,nan);
false_a(1,:,:) = false_a(1,:,:) + (rand(size(false_a(1,:,:)))-0.5)*max(max(max(swarm_ran(1,:,:))))*2;
false_a(2,:,:) = false_a(2,:,:) + (rand(size(false_a(2,:,:)))-0.5)*max(max(max(swarm_ran(2,:,:))))*2;
false_a(3,:,:) = false_a(3,:,:) + (rand(size(false_a(3,:,:)))-0.5)*max(max(max(abs(swarm_ran(3,:,:)))))*2;

swarm_ran_fal = [swarm_ran false_a];

% 加噪声
swarm_ran_noi = swarm_ran_fal + (rand(size(swarm_ran_fal))-0.5)*0.1 + tar_start_range';       %tar_start 起始位置好像没什么用 0.01



figure; hold on
for ki = 1:tar_num
    plot3(squeeze(swarm_ran_noi(1,ki,:)),squeeze(swarm_ran_noi(2,ki,:)),...
        squeeze(swarm_ran_noi(3,ki,:)),'.','color',[rand rand rand]);
end
xlabel('方位/m');ylabel('俯仰/m');zlabel('距离/m');
title('加噪声后集群目标三维运动航迹')
%{
X = [0;0];
F = [1,Tr;0,1] ;
H = [1,0];
Q = eye(2,2)*0.01;
R = 1;
Z = squeeze(swarm_ran_noi(1,1,:))'; %取一维
real = squeeze(swarm_ran(1,1,:));
a_M = 0;
P=eye(2,2)*1;
x = zeros(2,length(Z));
for ii = 1:length(Z)
    X_=F*x(:,ii) + a_M;
    P_=F*P*F'+Q;
    K=P_*H'/(H*P_*H'+R);
    x(:,ii+1)=X_+K*(Z(:,ii)-H*X_);
    P=(eye(2)-K*H)*P_;
end
tr = Tr*100:Tr:nan*Tr;
figure;
plot(tr,x(1,100:nan));
hold on
plot(tr,Z(100:nan));
plot(tr,real(100:nan));
legend("滤波值","观测值","真实值")

error1 = sum(abs(Z(100:nan)-real(100:nan)'))/nan;
error2 = sum(abs(x(1,100:nan)-real(100:nan)'))/nan;
%}

figure; hold on

for ki = 1:nan
    plot3(swarm_ran_noi(1,:,ki),swarm_ran_noi(2,:,ki),swarm_ran_noi(3,:,ki),'.')
    drawnow;
%     pause(0.01)
end
%% 跟踪滤波
tar_star = swarm_ran(:,:,1) + tar_start_range.';
tar_num = 5;
tar_trace = zeros(3,tar_num,nan);
tar_trace(:,:,1) = tar_star;
detec_num = size(swarm_ran_noi,2);
detec_tar_match_falg = zeros(1,detec_num);
for ki = 2:nan
    detec_tar_match_falg = zeros(1,detec_num);
    for km = 1:tar_num
        tar_P_lastFrame = tar_trace(:,km,ki-1);
        % 进行当前帧目标遍历匹配 - 一个目标匹配后，不再进行其他目标匹配
        for kl = 1:detec_num
            if detec_tar_match_falg(kl) == 0
                detec_posi = swarm_ran_noi(:,kl,ki);
                rang_dif = norm(tar_P_lastFrame - detec_posi);
                if rang_dif < range_g   % 匹配上，进行跟踪值更新
                    detec_tar_match_falg(kl) = 1;
                    tar_trace(:,km,ki) = detec_posi;
                end
            end
        end
    end
end



figure; hold on
for ki = 1:nan
    plot3(tar_trace(1,:,ki),tar_trace(2,:,ki),tar_trace(3,:,ki),'.')
    drawnow;
%     pause(0.01)
end

swarm_ran1 = swarm_ran + tar_start_range.';
tar_ind = 2;
figure;plot3(squeeze(tar_trace(1,tar_ind,:)),squeeze(tar_trace(2,tar_ind,:)),squeeze(tar_trace(3,tar_ind,:)),'.')
hold on
plot3(squeeze(swarm_ran1(1,tar_ind,:)),squeeze(swarm_ran1(2,tar_ind,:)),squeeze(swarm_ran1(3,tar_ind,:)),'r.')





