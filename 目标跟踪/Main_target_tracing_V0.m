%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       ��Ŀ�����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc


%% ��������
Tr = 10e-2   ;

%% ���������
range_g = 1 ;   % ���پ�����

%% Ŀ�꺽���趨
tar_num = 5        ;   % ��ȺĿ�����
tar_v = [3 0 -10]*5          ;   % ��ȺĿ�������ٶȣ�ÿ��Ŀ���ٽ����ٶȷ��룬�Ի�ö����շֱ�
tar_start_range = [-1000 1000 5000];   % Ŀ����ʼ����
swarm_r = [(1:tar_num)*5;       %��άλ��
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
nan = 1024;     %������
swarm_ran = zeros(3,tar_num,nan);   %3ά*Ŀ��*ʱ��
for ki = 1:nan
    t = star_t+ki*Tr*1;
    v_r = tar_v*t;
    swarm_ran(:,:,ki) = swarm_r + swarm_v_a.*sin(2*pi*swarm_v_f*t) + v_r.';     %λ��
end



figure; hold on
for ki = 1:tar_num
    plot3(squeeze(swarm_ran(1,ki,:)),squeeze(swarm_ran(2,ki,:)),...
        squeeze(swarm_ran(3,ki,:)),'.','color',[rand rand rand]);
end
xlabel('��λ/m');ylabel('����/m');zlabel('����/m');
title('��ȺĿ����ά�˶�����')


% ���龯
false_a = zeros(3,4,nan);
false_a(1,:,:) = false_a(1,:,:) + (rand(size(false_a(1,:,:)))-0.5)*max(max(max(swarm_ran(1,:,:))))*2;
false_a(2,:,:) = false_a(2,:,:) + (rand(size(false_a(2,:,:)))-0.5)*max(max(max(swarm_ran(2,:,:))))*2;
false_a(3,:,:) = false_a(3,:,:) + (rand(size(false_a(3,:,:)))-0.5)*max(max(max(abs(swarm_ran(3,:,:)))))*2;

swarm_ran_fal = [swarm_ran false_a];

% ������
swarm_ran_noi = swarm_ran_fal + (rand(size(swarm_ran_fal))-0.5)*0.1 + tar_start_range';       %tar_start ��ʼλ�ú���ûʲô�� 0.01



figure; hold on
for ki = 1:tar_num
    plot3(squeeze(swarm_ran_noi(1,ki,:)),squeeze(swarm_ran_noi(2,ki,:)),...
        squeeze(swarm_ran_noi(3,ki,:)),'.','color',[rand rand rand]);
end
xlabel('��λ/m');ylabel('����/m');zlabel('����/m');
title('��������ȺĿ����ά�˶�����')
%{
X = [0;0];
F = [1,Tr;0,1] ;
H = [1,0];
Q = eye(2,2)*0.01;
R = 1;
Z = squeeze(swarm_ran_noi(1,1,:))'; %ȡһά
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
legend("�˲�ֵ","�۲�ֵ","��ʵֵ")

error1 = sum(abs(Z(100:nan)-real(100:nan)'))/nan;
error2 = sum(abs(x(1,100:nan)-real(100:nan)'))/nan;
%}

figure; hold on

for ki = 1:nan
    plot3(swarm_ran_noi(1,:,ki),swarm_ran_noi(2,:,ki),swarm_ran_noi(3,:,ki),'.')
    drawnow;
%     pause(0.01)
end
%% �����˲�
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
        % ���е�ǰ֡Ŀ�����ƥ�� - һ��Ŀ��ƥ��󣬲��ٽ�������Ŀ��ƥ��
        for kl = 1:detec_num
            if detec_tar_match_falg(kl) == 0
                detec_posi = swarm_ran_noi(:,kl,ki);
                rang_dif = norm(tar_P_lastFrame - detec_posi);
                if rang_dif < range_g   % ƥ���ϣ����и���ֵ����
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





