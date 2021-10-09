%%%%TAS函数%%%%
function [tar_find,tar_find_ang,tar_find_dis,tar_find_v,tar_posi,radar_posi,real_ang] = Tas_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita)
clc
getd = @(p)path(path,p);
getd('fire_control_subpro/');


%% 雷达系统仿真参数
c=3e8                       ;	% 光速
k=1.38e-23                  ;	% 玻尔兹曼常数
%Pt= 5000000                  ;	% 发射功率【W】
%Fc=10e8                     ;	% 中心频率【Hz】
lambda=c/Fc                 ;	% 工作波长【m】
%Tp=1e-4                    ;	% 脉冲宽度【微秒】
Fr=1/Tr                     ;	% 脉冲重复频率【Hz】
%B=1e6                       ;	% 带宽【Hz】
%Fs=0.1*Fc                      ;	% 采样率【Hz】
F=10^(5/10)                 ;	% 噪声系数
K=B/Tp                      ;	% 调频率【Hz】
%Tr=1/Fr                     ;	% 脉冲重复周期【秒】
Delta_t=1/Fs                ;	% 时域采样点时间间隔【秒】
Rx_d = 0.03;
R_max_range = Tr*c/2;

%% 天线参数
N_azi = 20                  ;   % 方位阵元数         %%%一个天线20*20,共2个天线
N_pit = 20                  ;   % 俯仰阵元数
ant_num = 4;
ant_pos = [Rx_d 0 Rx_d; Rx_d 0 -Rx_d; -Rx_d 0 -Rx_d; -Rx_d 0 Rx_d].';

%% 目标仿真参数
% 起始目标角度  这里的角度是相对飞机而言的
%Phi = [8 3 -5 ]./180*pi;                   %俯仰
%Sita = [-20 -1 15 ]./180*pi;             %方位
tar_num = length(Phi);                                %目标数
%{
%%%%
Pos_target=[ Pos_Aircraft+ 1e4*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))] ,...
    Pos_Aircraft+ 3e3*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...
    Pos_Aircraft+9e3*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))]];                   % 目标4
%   目标坐标【m】           X轴坐标              Y轴坐标                  Z轴坐标
V_target = [ [  1000;              2000;           5000] ,...
   [  -500;              1500;           -1000],...
   [  8000;              -3000;           4000]];    % 目标4
RCS_target=1*[1;1;1]; % 假定目标RCS慢起伏，即在仿真时间段内恒定
% 目标平均后向散射截面积【m^2】
%}
tra_lose = 0;               %
tar_find = 0;               %已发现目标个数
tar_find_ang = zeros(2,1);  %记录已发现目标的角度
tar_find_dis = zeros(4,1);  %记录已发现目标的距离
tar_find_v = zeros(3,1);    %记录已发现目标的速度
%% 上视搜索，搜索范围-60至+60
% 波束宽度设为5°，和差波束天线轴向夹角3° 方位向进行5°间隔扫描，方位向共扫描24个点
% 进行四行往复扫描
Beam_pit = [10 5 0 -5 -10]    ;       % 俯仰维扫描波束指向角  %%%4个角度
Bean_azi = [-6:6]*5       ;         % 方位维扫描波束指向角  %%%25个方位角 -60 —— +60
BeamShift = 2               ;       % 两个波束轴向夹角
TRA_CPI = 16;
CPI_num = 1               ;        % 每个波位驻留脉冲
% 每个CPI128个脉冲，一次扫描时间为4*25*128*Tr=1.28s
t_set_Tp=(Delta_t:Delta_t:Tp)'          ;	% 一个脉冲内的时间采样点数组
N=length(t_set_Tp);
%F_dolp = linspace(-Fr/2,Fr/2,CPI_num);      %多普勒频率
beam_wide = 10;                             %天线方向图范围

%% 卡尔曼滤波参数
x = [0;0;0;0;0;0];                                                                          %卡尔曼滤波初试状态
CPI_t = CPI_num*Tr;                                                                         %一个CPI总时间
F_kal = [1,0,0,Tr,0,0; 0,1,0,0,Tr,0; 0,0,1,0,0,Tr; 0,0,0,1,0,0; 0,0,0,0,1,0;0,0,0,0,0,1];   %状态转移矩阵
Q_kal = eye(6,6)*0.05;                                                                      %噪声干扰协方差矩阵
P = eye(6,6);                                                                               %状态协方差矩阵
R_kal = 0.1;                                                                                %观测噪声方差
u_kal = [0;0;0;0;0;0];                                                                      %控制向量
B_kal = [(Tr^2)/2;(Tr^2)/2;(Tr^2)/2;Tr;Tr;Tr];                                              %加速度向量
CPI_N = 1;                                                                                  %调试用
CPI_sum = 0;

radar_posi = Pos_Aircraft;
tar_posi = Pos_target;
%%  信号处理
disp('scan..')
tar_lose = 0;
for ki = 1:length(Beam_pit)
    pit_ang = Beam_pit(ki);                                                                                         %%%俯仰角
    for km = 1:length(Bean_azi)
        azi_ang = Bean_azi(km);                                                                                     %%%方位角
        fprintf('Azi_Beam=%d...Pit_Beam=%d...\n',azi_ang,pit_ang);
        [beam_1,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;      % 生成波束增益图
        [beam_2,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;      % 生成波束增益图
        [beam_3,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;
        [beam_4,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;
        
        %     Jet_posi = Pos_Aircraft + (ki-1)*length(Bean_azi)*CPI_num*Tr*V_Aircraft + (km-1)*CPI_num*Tr*V_Aircraft  ;   % 飞机在该波位起始位置   %%%每扫描一个角度位置变化一次
        %    Tar_posi = Pos_target + (ki-1)*length(Bean_azi)*CPI_num*Tr*V_Aircraft + (km-1)*CPI_num*Tr*V_target      ;   % 目标在该波位起始位置
        
        CPI_sig = zeros(tar_num,round(Tr*Fs),CPI_num);                                                              %记录整个CPI的回波数据
        tar_dist = zeros(tar_num,CPI_num);                                                                          %目标距离记录
        tar_v = zeros(tar_num,CPI_num);                                                                             %目标速度记录
        H_kal = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0]   ;                                                           %观测矩阵
        for kl = 1:CPI_num
            % 飞机位置与目标位置更新
            radar_posi = radar_posi + Tr*V_Aircraft;                                                               %每个波束飞机/目标都会运动一个单位距离
            tar_posi = tar_posi + Tr*V_target;                                                                   %目标位置更新
            tar_vec = zeros(ant_num,3,tar_num);
            for kk = 1:ant_num
                tar_vec(kk,:,:) = tar_posi - radar_posi - ant_pos(:,kk);                                            %飞机与目标的距离差
            end
            tar_zero = tar_posi - radar_posi;                                                                       %目标与原点的距离
            % 坐标系转换
            tar_posi_Spha = zeros(ant_num,3,tar_num);
            tar_posi_Spha_zero = zeros(3,tar_num);
            
            for kk = 1:tar_num
                for ii = 1:ant_num
                    [r,sita,phi] = Cart2Sphe(tar_vec(ii,1,kk),tar_vec(ii,2,kk),tar_vec(ii,3,kk));                   %转球坐标
                    tar_posi_Spha(ii,:,kk) = [r,sita,phi];                                                          %四个目标
                end
                [r,sita,phi] = Cart2Sphe(tar_zero(1,kk),tar_zero(2,kk),tar_zero(3,kk));                             %转球坐标
                tar_posi_Spha_zero(:,kk) = [r,sita,phi];
                tar_dist(kk,kl) = r;
                if(kl > 1)
                    tar_v(kk,kl) = (tar_dist(kk,kl)-tar_dist(kk,kl-1))/Tr;                                          %真实速度
                end
            end
            % 逐个目标进行回波仿真
            echo = zeros(round(Tp*Fs),4);                                                                              %Tp内
            s_echo = zeros(round(Tr*Fs),4);                                                                            %整个Tr
            s_match = zeros(round(Tr*Fs),4);
            %% 回波生成
            for kk = 1:tar_num
                for ii = 1:ant_num
                    tar_dely(ii) = tar_posi_Spha(ii,1,kk)*2/c;                                                      %各目标回波延时
                    Lp(ii)=1./((4*pi)^2*tar_posi_Spha(ii,1,kk).^4);                                                 %目标处的电磁波损耗
                end
                Gt(1) = GetBeamGain(beam_1,a1,p1,tar_posi_Spha(1,2,kk),tar_posi_Spha(1,3,kk),pit_ang,azi_ang,beam_wide);   %%%回波生成
                Gt(2) = GetBeamGain(beam_2,a1,p1,tar_posi_Spha(2,2,kk),tar_posi_Spha(2,3,kk),pit_ang,azi_ang,beam_wide);   %%%回波生成
                Gt(3) = GetBeamGain(beam_3,a1,p1,tar_posi_Spha(3,2,kk),tar_posi_Spha(3,3,kk),pit_ang,azi_ang,beam_wide);   %%%回波生成
                Gt(4) = GetBeamGain(beam_4,a1,p1,tar_posi_Spha(4,2,kk),tar_posi_Spha(4,3,kk),pit_ang,azi_ang,beam_wide);   %%%回波生成
                zero_dely = tar_posi_Spha_zero(1,kk)*2/c;
                Gr = Gt;                                                                                                   %这里假设接收增益与发射增益一致
                Echo_start=round((zero_dely)*Fs);                                                                          %目标回波起始采样点
                Echo_stop=Echo_start+N-1;                                                                                  %目标回波结束采样点                                                                           %匹配结果
                for ii = 1:ant_num
                    Magnitude_echo(ii)=sqrt(Pt*RCS_target(kk)*(Gt(1)+Gt(2)+Gt(3)+Gt(4))*Gr(ii)*lambda^2/(4*pi)*Lp(ii));    %波束A中目标回波幅度
                    echo(:,ii) = echo(:,ii) + Magnitude_echo(ii)*exp(2*pi*1i*Fc*( t_set_Tp-tar_dely(ii) ) ).*exp(1i*pi*K*(t_set_Tp - tar_dely(ii)).^2);         %四个天线的回波
                    %  s_echo(1:N,ii) = echo(:,ii);
                    % figure;plot(linspace(0,Tr,round(Tr*Fs)),real(s_echo(:,ii)))
                    %   s_echo(:,ii) = s_echo(:,ii) + 5000*sqrt(k*B*F*290/2)*( randn(size(s_echo(:,ii) ) )+1i*randn(size(s_echo(:,ii) ) ) );           %加噪声
                    %    figure;subplot(1,2,1);plot(linspace(0,Tr,round(Tp*Fs)),real(echo(:,ii)))
                    % [res] = bw_filter(s_echo(:,ii),B,Fc,Fs);
                    %[s_match(:,ii),R_lin,a_lin] = MatchFilter(s_echo(:,ii),Tr,Fs,Tp,Fc,K);                                    %匹配滤波
                    % subplot(1,2,2);plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,ii)))
                end
            end
            %% 回波处理
            tar_record = zeros(1,4);        %记录四个天线各回波的波峰位置
            Pfa = 1e-6;
            for ii = 1:ant_num
                s_echo(1:N,ii) = echo(:,ii);
                s_echo(:,ii) = s_echo(:,ii) + 0*sqrt(k*B*F*290/2)*( randn(size(s_echo(:,ii) ) )+1i*randn(size(s_echo(:,ii) ) ) );           %加噪声
                [s_match(:,ii),R_lin,a_lin] = MatchFilter(s_echo(:,ii),Tr,Fs,Tp,Fc,K,B);
                [XT,D,inx,iny] = my_Cfar(Pfa,s_match(:,ii),ceil(length(s_match(:,ii))/200),ceil(length(s_match(:,ii))/30));     %cfar检测
                %  figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,ii)))
                now_num = 1;
                is_tar = abs(s_match(iny,ii));
                %    figure;plot(1:length(is_tar),is_tar);
                if(~isempty(inx))
                    while(1)
                        [max_x,max_y] = find(is_tar == max(is_tar));
                        now_max = is_tar(max_x,max_y);
                        tar_record(now_num,ii) = iny(max_x);
                        now_num = 1 + now_num;
                        is_tar = del_panban(is_tar,max_x);
                        if(sum(is_tar) == 0)
                            break
                        end
                    end
                end
            end
            s_Sigma=zeros(round(Tr*Fs),1);                                              % 定义和通道信号数组
            s_Delta_yaw=zeros(round(Tr*Fs),1);                                          % 定义差通道信号数组
            s_Delta_pit=zeros(round(Tr*Fs),1);
            for ii = 1:size(tar_record,1)
                if(sum(tar_record(ii,:)) ~= 0)
                    if(abs(tar_record(ii,1) - tar_record(ii,2))+abs(tar_record(ii,1)-tar_record(ii,3))+abs(tar_record(ii,1)-tar_record(ii,4))< 20)      %查看四组天线检测到结果是否一致
                        s_Sigma = s_match(tar_record(ii,1),1) + s_match(tar_record(ii,2),2) + s_match(tar_record(ii,3),3) + s_match(tar_record(ii,4),4);
                        s_Delta_yaw = s_match(tar_record(ii,1),1) + s_match(tar_record(ii,2),2) - s_match(tar_record(ii,3),3) - s_match(tar_record(ii,4),4);
                        s_Delta_pit = s_match(tar_record(ii,1),1) - s_match(tar_record(ii,2),2) - s_match(tar_record(ii,3),3) + s_match(tar_record(ii,4),4);
                        %s_Sigma = s_match(:,1) + s_match(:,2) + s_match(:,3) + s_match(:,4);
                        %s_Delta_yaw = s_match(:,1) + s_match(:,2) - s_match(:,3) - s_match(:,4);
                        %s_Delta_pit = s_match(:,1) - s_match(:,2) - s_match(:,3) + s_match(:,4);
                        tar_dis = sum(tar_record(ii,:))/4/round(Tr*Fs)/2*Tr*c;                        %测距
                        [in1,in2] = find(s_Sigma == max(s_Sigma));                          %找最大值作为测角点
                        %% 测角
                        bi_yaw = imag(s_Delta_yaw(in1,in2)/s_Sigma(in1,in2));
                        bi_pit = imag(s_Delta_pit(in1,in2)/s_Sigma(in1,in2));
                        pit = atan(bi_pit)*lambda/pi/Rx_d/4;
                        theta_pit = asin(pit)*180/pi
                        yaw = atan(bi_yaw)*lambda/pi/4/Rx_d/cos(theta_pit*pi/180);
                        theta_yaw = asin(yaw)*180/pi
                        
                        for tar_c = 1:tar_num
                            real_range(tar_c) = sqrt(tar_zero(1,tar_c)^2+tar_zero(2,tar_c)^2 + tar_zero(3,tar_c)^2);
                            real_ang(1,tar_c) = atan(tar_zero(1,tar_c)/tar_zero(2,tar_c))*180/pi;                            %方位
                            real_ang(2,tar_c) = atan(tar_zero(3,tar_c)/sqrt(tar_zero(2,tar_c)^2 + tar_zero(1,tar_c)^2))*180/pi; %俯仰
                        end
                        real_ang    %真实角度;
                        %% 记录已发现目标
                        if(tar_find == 0)
                            tar_find = 1 + tar_find;
                            tar_find_ang(:,tar_find) = [theta_pit;theta_yaw];
                            tar_find_dis(1,tar_find) = tar_dis;
                        else
                            tar_ang_sigma_max = 0;
                            tar_ang_sigma_min = 10000;
                            tar_dis_sigma_max = 0;
                            tar_dis_sigma_min = 100000;
                            
                            %判断是否添加新发现的目标
                            for tar_c = 1:tar_find
                                tar_ang_sigma = norm(abs(tar_find_ang(:,tar_c) - [theta_pit;theta_yaw]));
                                tar_dis_sigma = abs(tar_find_dis(1,tar_c) - tar_dis);
                                if(tar_ang_sigma_max < tar_ang_sigma)
                                    tar_ang_sigma_max = tar_ang_sigma;
                                end
                                if(tar_ang_sigma_min > tar_ang_sigma)
                                    tar_ang_sigma_min = tar_ang_sigma;
                                    tar_ang_min = tar_c;
                                end
                                if(tar_dis_sigma_max < tar_dis_sigma)
                                    tar_dis_sigma_max = tar_dis_sigma;
                                end
                                if(tar_dis_sigma_min > tar_dis_sigma)
                                    tar_dis_sigma_min = tar_dis_sigma;
                                    tar_dis_min = tar_c;
                                end
                            end
                            
                            if(tar_ang_sigma_min < 3 && tar_dis_sigma_min < 100 )
                                tar_find_ang(:,tar_ang_min) = [theta_pit;theta_yaw];                            %若和已发现的目标相关，则更新该目标信息
                                tar_find_dis(:,tar_ang_min) = [tar_dis;radar_posi + [tar_dis*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);tar_dis*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);tar_dis*sin(theta_pit*pi/180)]];
                            elseif(tar_ang_sigma_min > 3 && tar_dis_sigma_min > 50 )   %若与已发现目标无关，则添加为新目标
                                tar_find = 1 + tar_find;
                                tar_find_ang(:,tar_find) = [theta_pit;theta_yaw];
                                tar_find_dis(:,tar_find) = [tar_dis;radar_posi + [tar_dis*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);tar_dis*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);tar_dis*sin(theta_pit*pi/180)]];
                            end
                        end
                        
                        %    tar_rock = 1;
                        %   now_pit = Beam_pit(ki);
                        %  now_yaw = Bean_azi(km);
                        % last_obe = obe_val(:,kk);
                        %break;
                    else
                        disp('noise')
                        
                    end
                else
                    disp('nothing')
                    
                end
            end
            %figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,1)))
        end
    end
end

%% 跟踪
if(tar_find > 0)
    tar_de_flag = zeros(tar_find,1);
    x_record1 = zeros(tar_find,6,TRA_CPI*CPI_N);                                                               %滤波输出记录
    dis_rec1 = zeros(tar_find,3,TRA_CPI*CPI_N);                                                                %测量距离记录
    real_dis = zeros(tar_num,3,TRA_CPI*CPI_N);
    ang = zeros(tar_find,2,TRA_CPI*CPI_N);
    meas_dis = zeros(tar_find,TRA_CPI);                                                          %目标距离记录
    obe_val = zeros(3,tar_find);                                                                 %观测值记录
    tar_lose_num = 1;                                                                            %记录丢失目标的个数
    for kk = 1:tar_find
        x_tar(:,kk) = x;                                                                          %四个目标
        P_kal(:,:,kk) = P;
    end
    CPI_sig = zeros(tar_find,round(Tr*Fs),TRA_CPI);                                                              %记录整个CPI的回波数据
    tar_lose_rec = [];
    for tar_c = 1:tar_find
        pit_ang = tar_find_ang(1,tar_c);
        azi_ang = tar_find_ang(2,tar_c);
        for T = 1:CPI_N
            disp('tracing...')
            [beam_1,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;      % 生成波束增益图
            [beam_2,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;      % 生成波束增益图
            [beam_3,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;
            [beam_4,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;
            tar_dist = zeros(tar_find,TRA_CPI);                                                                          %目标距离记录
            tar_v = zeros(tar_find,TRA_CPI);                                                                             %目标速度记录
            obe_v = zeros(3,tar_find,CPI_num);
            H_kal = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0]   ;                                                           %观测矩阵
            for kl = 1:TRA_CPI
                echo = zeros(round(Tp*Fs),4);                                                                              %Tp内
                s_echo = zeros(round(Tr*Fs),4);                                                                            %整个Tr
                s_match = zeros(round(Tr*Fs),4);
                % 飞机位置与目标位置更新
                radar_posi = radar_posi + Tr*V_Aircraft;                                                               %每个波束飞机/目标都会运动一个单位距离
                tar_posi = tar_posi + Tr*V_target;                                                                   %目标位置更新
                tar_vec = zeros(ant_num,3,tar_num);
                % real_dis(:,kl+CPI_sum*TRA_CPI) = tar_posi;
                real_dis(:,:,kl) = tar_posi;
                for kk = 1:ant_num
                    tar_vec(kk,:,:) = tar_posi - radar_posi - ant_pos(:,kk);                                            %天线与目标的距离差
                end
                tar_zero = tar_posi - radar_posi;                                                                       %目标与原点的距离
                % 坐标系转换
                tar_posi_Spha = zeros(ant_num,3,tar_num);
                tar_posi_Spha_zero = zeros(3,tar_num);
                
                for kk = 1:tar_num
                    for ii = 1:ant_num
                        [r,sita,phi] = Cart2Sphe(tar_vec(ii,1,kk),tar_vec(ii,2,kk),tar_vec(ii,3,kk));                   %转球坐标
                        tar_posi_Spha(ii,:,kk) = [r,sita,phi];                                                          %四个目标
                    end
                    [r,sita,phi] = Cart2Sphe(tar_zero(1,kk),tar_zero(2,kk),tar_zero(3,kk));                             %转球坐标
                    tar_posi_Spha_zero(:,kk) = [r,sita,phi];
                    tar_dist(kk,kl) = r;
                    if(kl > 1)
                        tar_v(kk,kl) = (tar_dist(kk,kl)-tar_dist(kk,kl-1))/Tr;                                          %真实速度
                    end
                end
                % 所有目标进行回波叠加
                for kk = 1:tar_num
                    for ii = 1:ant_num
                        tar_dely(ii) = tar_posi_Spha(ii,1,kk)*2/c;                                                      %各目标回波延时
                        Lp(ii)=1./((4*pi)^2*tar_posi_Spha(ii,1,kk).^4);                                                 %目标处的电磁波损耗
                    end
                    Gt(1) = GetBeamGain(beam_1,a1,p1,tar_posi_Spha(1,2,kk),tar_posi_Spha(1,3,kk),pit_ang,azi_ang,beam_wide);   %%%回波生成
                    Gt(2) = GetBeamGain(beam_2,a1,p1,tar_posi_Spha(2,2,kk),tar_posi_Spha(2,3,kk),pit_ang,azi_ang,beam_wide);   %%%回波生成
                    Gt(3) = GetBeamGain(beam_3,a1,p1,tar_posi_Spha(3,2,kk),tar_posi_Spha(3,3,kk),pit_ang,azi_ang,beam_wide);   %%%回波生成
                    Gt(4) = GetBeamGain(beam_4,a1,p1,tar_posi_Spha(4,2,kk),tar_posi_Spha(4,3,kk),pit_ang,azi_ang,beam_wide);   %%%回波生成
                    zero_dely = tar_posi_Spha_zero(1,kk)*2/c;
                    Gr = Gt;                                                                                                   %这里假设接收增益与发射增益一致
                    Echo_start=round((zero_dely)*Fs);                                                                          %目标回波起始采样点
                    Echo_stop=Echo_start+N-1;                                                                                  %目标回波结束采样点                                                                           %匹配结果
                    for ii = 1:ant_num
                        Magnitude_echo(ii)=sqrt(Pt*RCS_target(kk)*(Gt(1)+Gt(2)+Gt(3)+Gt(4))*Gr(ii)*lambda^2/(4*pi)*Lp(ii));    %波束A中目标回波幅度
                        echo(:,ii) = echo(:,ii) + Magnitude_echo(ii)*exp(2*pi*1i*Fc*( t_set_Tp-tar_dely(ii) ) ).*exp(1i*pi*K*(t_set_Tp - tar_dely(ii)).^2);         %四个天线的回波
                    end
                end
                Pfa = 1e-6;
                for ii = 1:ant_num
                    s_echo(1:N,ii) = echo(:,ii);
                    s_echo(:,ii) = s_echo(:,ii) + 0*sqrt(k*B*F*290/2)*( randn(size(s_echo(:,ii) ) )+1i*randn(size(s_echo(:,ii) ) ) );           %加噪声
                    [s_match(:,ii),R_lin,a_lin] = MatchFilter(s_echo(:,ii),Tr,Fs,Tp,Fc,K,B);
                    %       [D,inx,iny] = my_Cfar(Pfa,s_match(:,ii),ceil(length(s_match(:,ii))/100),ceil(length(s_match(:,ii))/80));     %cfar检测
                    % figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,ii)))
                    
                end
                [XT1,D1,inx1,iny1] = my_Cfar(Pfa,s_match(:,1),ceil(length(s_match(:,1))/200),ceil(length(s_match(:,1))/20));     %cfar检测
                [XT2,D2,inx2,iny2] = my_Cfar(Pfa,s_match(:,2),ceil(length(s_match(:,2))/200),ceil(length(s_match(:,2))/20));
                [XT3,D3,inx3,iny3] = my_Cfar(Pfa,s_match(:,3),ceil(length(s_match(:,3))/200),ceil(length(s_match(:,3))/20));
                [XT4,D4,inx4,iny4] = my_Cfar(Pfa,s_match(:,4),ceil(length(s_match(:,4))/200),ceil(length(s_match(:,4))/20));
                
                now_index = tar_find_dis(1,tar_c)/(Tr/2)/c*round(Tr*Fs);
                now_index1 = find(iny1>now_index-300 & iny1 < now_index + 300);
                now_index2 = find(iny2>now_index-300 & iny2 < now_index + 300);
                now_index3 = find(iny3>now_index-300 & iny3 < now_index + 300);
                now_index4 = find(iny4>now_index-300 & iny4 < now_index + 300);
                now_iny1 = iny1(now_index1);now_iny2 = iny2(now_index2);now_iny3 = iny3(now_index3);now_iny4 = iny4(now_index4);
                s_Sigma=zeros(round(Tr*Fs),1);                                              % 定义和通道信号数组
                s_Delta_yaw=zeros(round(Tr*Fs),1);                                          % 定义差通道信号数组
                s_Delta_pit=zeros(round(Tr*Fs),1);
                if( ~isempty(now_iny1) && ~isempty(now_iny2) && ~isempty(now_iny3) && ~isempty(now_iny4))   %检测最大值
                    [max_x1,max_y1] = find(s_match == max(s_match(now_iny1,1)));
                    [max_x2,max_y2] = find(s_match == max(s_match(now_iny2,1)));
                    [max_x3,max_y3] = find(s_match == max(s_match(now_iny3,1)));
                    [max_x4,max_y4] = find(s_match == max(s_match(now_iny4,1)));
                    
                    if(abs(max_x1 - max_x2)+abs(max_x1-max_x3)+abs(max_x1-max_x4)< 100)      %查看四组天线检测到结果是否一致
                        s_Sigma = s_match(max_x1,1) + s_match(max_x2,2) + s_match(max_x3,3) + s_match(max_x4,4);
                        s_Delta_yaw = s_match(max_x1,1) + s_match(max_x2,2) - s_match(max_x3,3) - s_match(max_x4,4);
                        s_Delta_pit = s_match(max_x1,1) - s_match(max_x2,2) - s_match(max_x3,3) + s_match(max_x4,4);
                        %s_Sigma = s_match(:,1) + s_match(:,2) + s_match(:,3) + s_match(:,4);
                        %s_Delta_yaw = s_match(:,1) + s_match(:,2) - s_match(:,3) - s_match(:,4);
                        %s_Delta_pit = s_match(:,1) - s_match(:,2) - s_match(:,3) + s_match(:,4);
                        meas_dis(tar_c,kl) = (max_x1+max_x2+max_x3+max_x4)/4/round(Tr*Fs)/2*Tr*c;                        %测距
                        [in1,in2] = find(s_Sigma == max(s_Sigma));                          %找最大值作为测角点
                        %% 测角
                        bi_yaw = imag(s_Delta_yaw(in1,in2)/s_Sigma(in1,in2));
                        bi_pit = imag(s_Delta_pit(in1,in2)/s_Sigma(in1,in2));
                        pit = atan(bi_pit)*lambda/pi/Rx_d/4;
                        theta_pit = asin(pit)*180/pi;
                        yaw = atan(bi_yaw)*lambda/pi/4/Rx_d/cos(theta_pit*pi/180);
                        theta_yaw = asin(yaw)*180/pi;
                        % ang(:,kl+CPI_sum*TRA_CPI) = [theta_pit;theta_yaw];
                        ang(tar_c,:,kl) = [theta_pit;theta_yaw];
                        tar_find_ang(1,tar_c) = theta_pit;
                        tar_find_ang(2,tar_c) = theta_yaw;
                        for ii = 1:tar_num
                            real_range(ii) = sqrt(tar_zero(1,ii)^2+tar_zero(2,ii)^2 + tar_zero(3,ii)^2);
                            real_ang(1,ii) = atan(tar_zero(1,ii)/tar_zero(2,ii))*180/pi;                            %方位
                            real_ang(2,ii) = atan(tar_zero(3,ii)/sqrt(tar_zero(2,ii)^2 + tar_zero(1,ii)^2))*180/pi; %俯仰
                        end
                        real_ang;    %真实角度
                        
                        %% 卡尔曼滤波
                        CPI_sig(tar_c,:,kl) = s_match(:,1);
                        last_obe(:,tar_c) = obe_val(:,tar_c);
                        obe_val(:,tar_c) = radar_posi + [meas_dis(tar_c,kl)*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);meas_dis(tar_c,kl)*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);meas_dis(tar_c,kl)*sin(theta_pit*pi/180)]; %距离观察值
                        if(kl>1)
                            obe_v(:,tar_c,kl) = (obe_val(:,tar_c) - last_obe(:,tar_c))/Tr;
                        else
                            obe_v(:,tar_c,kl) = 0;
                        end
                        kal_obe = [obe_val(:,tar_c)];
                        [x_tar(:,tar_c),P_kal(:,:,tar_c)] = my_kalman_filter(x_tar(:,tar_c),P_kal(:,:,tar_c),H_kal,F_kal,R_kal,Q_kal,u_kal,B_kal,kal_obe);
                        % x_record1(:,kl+CPI_sum*TRA_CPI) = x_tar(:,1);       %记录目标的6个量的跟踪结果
                        % dis_rec1(:,kl+CPI_sum*TRA_CPI) = obe_val(:,1);      %记录观察值
                        x_record1(tar_c,:,kl) = x_tar(:,1);
                        dis_rec1(tar_c,:,kl) = obe_val(:,tar_c);
                    else
                        disp('noise')
                        tar_de_flag(tar_c,1) = tar_de_flag(tar_c,1) + 1;
                        CPI_sig(tar_c,:,kl) = 0;
                    end
                else
                    disp('nothing')
                    tar_de_flag(tar_c,1) = tar_de_flag(tar_c,1) + 1;
                    CPI_sig(tar_c,:,kl) = 0;
                    
                end
            end
            CPI_sum = CPI_sum + 1;
        end
        x_soft = zeros(3,TRA_CPI*CPI_N);
        for kk = 1:3
            x_soft(kk,:) = soft_fil(dis_rec1(tar_c,kk,:),8);
        end
        total_data(:,tar_c,:) = x_soft;
        
    end
    %增加虚警点
    false_a = zeros(3,tar_find,TRA_CPI*CPI_N);
    for kk = 1:tar_find
        [x1] = find(abs(total_data(1,kk,:)) == max(max(max(abs(total_data(1,kk,:))))));
        [x2] = find(abs(total_data(2,kk,:)) == max(max(max(abs(total_data(2,kk,:))))));
        [x3] = find(abs(total_data(3,kk,:)) == max(max(max(abs(total_data(3,kk,:))))));
        false_a(1,kk,:) = false_a(1,kk,:) + sign(total_data(1,kk,x1(1)))*(rand(size(false_a(1,kk,:)))+0.1)*max(max(abs(total_data(1,kk,:))))*2;
        false_a(2,kk,:) = false_a(2,kk,:) + sign(total_data(2,kk,x2(1)))*(rand(size(false_a(2,kk,:)))+0.1)*max(max(abs(total_data(2,kk,:))))*2;
        false_a(3,kk,:) = false_a(3,kk,:) + sign(total_data(3,kk,x3(1)))*(rand(size(false_a(3,kk,:)))+0.1)*max(max(abs(total_data(3,kk,:))))*2;
        total_data(:,tar_find+kk,:) = false_a(:,kk,:);
    end
    
    
    figure;hold on
    for kk = 1:TRA_CPI*CPI_N
        plot3(squeeze(total_data(1,:,kk)),squeeze(total_data(2,:,kk)),squeeze(total_data(3,:,kk)),'.','color',[rand rand rand])
        drawnow;
    end
    xlabel('x');ylabel('y');zlabel('z');
    
    %% 跟踪滤波
    % 跟踪参数
    dis_door = 8;                           %距离门大小
    if_rock = zeros(1,TRA_CPI*CPI_N);       %是否跟踪到
    lose_num = 0;                           %跟丢累计数
    pre_num = 5;                           %预测参考数
    for tar_c = 1:tar_find
        tra_pro = zeros(3,TRA_CPI*CPI_N);       %跟踪结果
        start_point = total_data(:,tar_c,1);        %跟踪起点确认
        tra_pro(:,1) = start_point;
        for ii = 2:TRA_CPI*CPI_N
            for kk = 1:size(total_data,2)
                now = total_data(:,kk,ii);                  %当前判断点
                sigma = norm(abs(now - tra_pro(:,ii-1)));        %算三个距离总共平方差
                if(sigma < dis_door)
                    tra_pro(:,ii) = total_data(:,kk,ii);    %确定点
                    if_rock(ii) = 1;
                    lose_num = 0;
                    break
                end
            end
            if(if_rock(ii)==0)
                if(ii > pre_num)
                    tra_pro(:,ii) = my_predict(tra_pro(:,ii-pre_num:ii-1));
                elseif(ii > 2 && ii < pre_num)
                    tra_pro(:,ii) = my_predict(tra_pro(:,1:ii-1));
                else
                    tra_pro(:,ii) = tra_pro(:,ii-1);
                end
                lose_num = lose_num + 1;
            end
            if(lose_num > 2)
                tar_lose = 1;                               %跟丢了，继续搜索
                disp('lose')
                break
            end
        end
        if(tar_lose == 1)
            tar_find = tar_find - 1;
            tar_lose_rec(tar_lose_num) = tar_c;
            tar_lose_num = 1 + tar_lose_num;
        end
        figure;hold on
        for kk = 1:TRA_CPI*CPI_N
            plot3(tra_pro(1,kk),tra_pro(2,kk),tra_pro(3,kk),'.')
            %drawnow
        end
        xlabel('x');ylabel('y');zlabel('z');
        hold off
        
        
        total_dis_cha = [0;0;0];
        for kk = 2:TRA_CPI*CPI_N
            total_dis_cha = tra_pro(:,kk) - tra_pro(:,kk-1) + total_dis_cha;
        end
        tar_find_v(:,tar_c) = sum(total_dis_cha,2)/(TRA_CPI*CPI_N-1)/Tr;
    end
    
    
    
    %% mtd测速
    ff = linspace(-Fr/2,Fr/2,TRA_CPI);                                      %多普勒频谱范围
    RD_sig = zeros(tar_find,TRA_CPI);                                        %存储慢时间fft
    s_max = zeros(tar_find,1);
    for ii = 1:tar_find
        if(tar_de_flag(ii,1) < TRA_CPI/2)
            [s_max] = find(abs(CPI_sig(ii,:,1)) == max(abs(CPI_sig(ii,:,1))));        %找距离上的最大值
            % figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(CPI_sig(ii,:,1)))
            RD_sig(ii,:) = fftshift(fft(CPI_sig(ii,s_max(1),:)));              %对距离上最大值做fft
            %figure;plot(ff,abs(RD_sig(ii,:)))
            [ff_max] = find(abs(RD_sig(ii,:)) == max(abs(RD_sig(ii,:))));   %找频谱最大值
            fd = ff(ff_max);
            tar_v(ii,1);                                                    %真实速度
            vd(ii,1) = -fd*lambda/2;                                              %测速结果
            %H_kal = eye(6,6);
            % obe_val2 = [obe_val(1,kk);obe_val(2,kk);obe_val(3,kk);]
            %  [x_tar(:,kk),P_tar(:,:,kk)] = my_kalman_filter(x_tar(:,kk),P_tar(:,:,kk),H_kal,F_kal,R_kal,Q_kal,u_kal,B_kal,obe_val(:,kk));
            %x = my_kalman_filter()
        end
    end
    if(~isempty(tar_lose_rec))
        for ii = 1:length(tar_lose_rec)
            tar_find_ang(:,tar_lose_rec(ii)) = [];      %跟丢了就把该目标相关数据删除
            tar_find_dis(:,tar_lose_rec(ii)) = [];
            tar_find = tar_find - 1;
        end
    end
end
