%%%%%TWS2%%%%
clear;
close all
clc
getd = @(p)path(path,p);
getd('fire_control_subpro/');
%% 载机平台参数
Pos_Aircraft=[0;0;0]        ;   % 载机三维坐标【m】
V_Aircraft=[0;0;0]          ;	% 载机速度矢量【m/s】

%% 雷达系统仿真参数
c=3e8                       ;	% 光速
k=1.38e-23                  ;	% 玻尔兹曼常数
Pt= 5000000                 ;	% 发射功率【W】
Fc=10e8                     ;	% 中心频率【Hz】
lambda=c/Fc                 ;	% 工作波长【m】
Tp=1e-4                     ;	% 脉冲宽度【微秒】
Fr=5*1e3                    ;	% 脉冲重复频率【Hz】
B=1e6                       ;	% 带宽【Hz】
Fs=0.01*Fc                    ;	% 采样率【Hz】   采样率与测角精度有关
F=10^(5/10)                 ;	% 噪声系数
K=B/Tp                      ;	% 调频率【Hz】
Tr=1/Fr                     ;	% 脉冲重复周期【秒】
Delta_t=1/Fs                ;	% 时域采样点时间间隔【秒】
Rx_d = 0.005                ;   % 天线组直接的距离，注意和波长的长度关系

%% 天线参数
N_azi = 20                  ;   % 方位阵元数         %%%一个天线20*20,共2个天线
N_pit = 20                  ;   % 俯仰阵元数
ant_num = 4;
ant_pos = [Rx_d 0 Rx_d; Rx_d 0 -Rx_d; -Rx_d 0 -Rx_d; -Rx_d 0 Rx_d].';

%% 目标仿真参数
% 起始目标角度  这里的角度是相对飞机而言的
Phi = [5 2 0 -10]./180*pi;                   %俯仰
Sita = [-28 -3 15 -15]./180*pi;             %方位
tar_num = 4;                                %目标数
%%%%
Pos_target=[ Pos_Aircraft+ 3.50e3*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))],...             % 目标1
            Pos_Aircraft+ 2e3*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...                 % 目标2
            Pos_Aircraft+8e3*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))],...                  % 目标3
            Pos_Aircraft+9e3*[cos(Phi(4))*sin(Sita(4)); cos(Phi(4))*cos(Sita(4)); sin(Phi(4))]];                    % 目标4
%   目标坐标【m】           X轴坐标              Y轴坐标                  Z轴坐标
V_target = [ [ 2000;              3000;           5000],...     % 目标2         %%%速度
    [    1000;              -5000;           3000],...          % 目标2
    [    4000;              -1900;           8000],...          % 目标3
    [    -2000;              2000;             2000] ];         % 目标4
RCS_target=1*[1;1;1;1];                                         % 假定目标RCS慢起伏，即在仿真时间段内恒定
% 目标平均后向散射截面积【m^2】

%% 上视搜索，搜索范围-60至+60
% 波束宽度设为5°，和差波束天线轴向夹角3° 方位向进行5°间隔扫描，方位向共扫描24个点
% 进行四行往复扫描
Beam_pit = [3  ];%-5 -10]    ;          % 俯仰维扫描波束指向角  %%%4个角度
Bean_azi = [-6:3]*5       ;                 % 方位维扫描波束指向角  %%%25个方位角 -60 —— +60
BeamShift = 2               ;               % 两个波束轴向夹角
CPI_num = 64               ;                % 每个波位驻留脉冲
% 每个CPI128个脉冲，一次扫描时间为4*25*128*Tr=1.28s
t_set_Tp=(Delta_t:Delta_t:Tp)'          ;	% 一个脉冲内的时间采样点数组
N=length(t_set_Tp);
F_dolp = linspace(-Fr/2,Fr/2,CPI_num);      %多普勒频率
beam_wide = 20;                             %天线方向图范围

%% 卡尔曼滤波参数 
x = [0;0;0;0;0;0];                                                                          %卡尔曼滤波初试状态
x_tar = [x,x,x,x];                                                                          %四个目标
CPI_t = CPI_num*Tr;                                                                         %一个CPI总时间
F_kal = [1,0,0,Tr,0,0; 0,1,0,0,Tr,0; 0,0,1,0,0,Tr; 0,0,0,1,0,0; 0,0,0,0,1,0;0,0,0,0,0,1];   %状态转移矩阵
Q_kal = eye(6,6)*0.00001;                                                                   %噪声干扰协方差矩阵
P = eye(6,6);                                                                               %状态协方差矩阵
P_kal(:,:,1) = P;P_kal(:,:,2) = P;P_kal(:,:,3) = P;P_kal(:,:,4) = P;                        
R_kal = 0.1;                                                                                %观测噪声方差
u_kal = [0;0;0;0;0;0];                                                                      %控制向量
B_kal = [(Tr^2)/2;(Tr^2)/2;(Tr^2)/2;Tr;Tr;Tr];                                              %加速度向量


obe_val = zeros(3,tar_num);                                                                 %观测值记录
last_obe = zeros(3,tar_num);
meas_dis = zeros(tar_num,1);                                                                
x_record1 = zeros(6,CPI_num);                                                               %滤波输出记录
x_record2 = zeros(6,CPI_num);
x_record3 = zeros(6,CPI_num);
x_record4 = zeros(6,CPI_num);
dis_rec1 = zeros(3,CPI_num);                                                                %测量距离记录
dis_rec2 = zeros(3,CPI_num);
dis_rec3 = zeros(3,CPI_num);
dis_rec4 = zeros(3,CPI_num);
tar_find_ang = zeros(2,1);                                                                  %记录已发现目标的角度
tar_find_dis = zeros(1,1);                                                                  %记录已发现目标的距离
T_cnt = 1;                                                                                  %第几个发射周期

radar_posi = Pos_Aircraft;
tar_posi = Pos_target;
tar_find = 0;                                                                               %已发现目标个数

while(1)
        tar_find_ang_rec = zeros(4,2,CPI_num*5*25);
        tar_find_dis_rec = zeros(4,3,CPI_num*5*25);
        disp('scan..')
        for ki = 1:length(Beam_pit)
            pit_ang = Beam_pit(ki);                                                                                         %%%俯仰角
            for km = 1:length(Bean_azi)
                tar_lose_rec = [];
                tar_lose_num = 1;                                                                                           %记录丢失目标的个数
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
                    radar_posi = radar_posi + Tr*V_Aircraft;                                                                %每个波束飞机/目标都会运动一个单位距离
                    tar_posi = tar_posi + Tr*V_target;                                                                      %目标位置更新
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
                    echo = zeros(round(Tp*Fs),4);                                                                           %Tp内
                    s_echo = zeros(round(Tr*Fs),4);                                                                         %整个Tr
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
                            echo(:,ii) = echo(:,ii) + Magnitude_echo(ii)*exp(2*pi*1i*Fc*( t_set_Tp-tar_dely(ii) ) ).*exp(1i*pi*K*(t_set_Tp - tar_dely(ii)).^2);     %四个天线的回波
                        end
                    end
                    %% 回波处理
                    tar_record = zeros(1,4);                                                                                       %记录四个天线各回波的波峰位置
                    Pfa = 1e-6;
                    for ii = 1:ant_num
                        s_echo(1:N,ii) = echo(:,ii);
                        s_echo(:,ii) = s_echo(:,ii) + 0*sqrt(k*B*F*290/2)*( randn(size(s_echo(:,ii) ) )+1i*randn(size(s_echo(:,ii) ) ) );           %加噪声
                        [s_match(:,ii),R_lin,a_lin] = MatchFilter(s_echo(:,ii),Tr,Fs,Tp,Fc,K,B);
                        [XT,D,inx,iny] = my_Cfar(Pfa,s_match(:,ii),ceil(length(s_match(:,ii))/100),ceil(length(s_match(:,ii))/20));                 %cfar检测
                          %figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,ii)))
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
                    s_Sigma=zeros(round(Tr*Fs),1);                                                          % 定义和通道信号数组
                    s_Delta_yaw=zeros(round(Tr*Fs),1);                                                      % 定义差通道信号数组
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
                                [in1,in2] = find(s_Sigma == max(s_Sigma));                                    %找最大值作为测角点
                                %% 测角
                                bi_yaw = imag(s_Delta_yaw(in1,in2)/s_Sigma(in1,in2));
                                bi_pit = imag(s_Delta_pit(in1,in2)/s_Sigma(in1,in2));
                                pit = atan(bi_pit)*lambda/pi/Rx_d/4;
                                theta_pit = asin(pit)*180/pi;
                                yaw = atan(bi_yaw)*lambda/pi/4/Rx_d/cos(theta_pit*pi/180);
                                theta_yaw = asin(yaw)*180/pi;

                                for tar_c = 1:tar_num
                                    real_range(tar_c) = sqrt(tar_zero(1,tar_c)^2+tar_zero(2,tar_c)^2 + tar_zero(3,tar_c)^2);
                                    real_ang(1,tar_c) = atan(tar_zero(1,tar_c)/tar_zero(2,tar_c))*180/pi;                               %方位
                                    real_ang(2,tar_c) = atan(tar_zero(3,tar_c)/sqrt(tar_zero(2,tar_c)^2 + tar_zero(1,tar_c)^2))*180/pi; %俯仰
                                end
                                real_ang                     ;                                                                          %真实角度;
                                %% 记录已发现目标
                                if(tar_find == 0)                                                                                       %未存在目标时直接把目标信息存入数据库
                                    tar_find = 1 + tar_find;
                                    tar_find_ang(:,tar_find) = [theta_pit;theta_yaw];
                                    tar_find_dis(1,tar_find) = tar_dis;
                                    tar_find_ang_rec(1,:,T_cnt) = [theta_pit;theta_yaw];
                                    tar_find_dis_rec(1,:,T_cnt) =  radar_posi + [tar_dis*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);tar_dis*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);tar_dis*sin(theta_pit*pi/180)];
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
                                    
                                    if(tar_ang_sigma_min < 2 && tar_dis_sigma_min < 50 && tar_dis_min == tar_ang_min)
                                        tar_find_ang(:,tar_ang_min) = [theta_pit;theta_yaw];    %若和已发现的目标相关，则更新该目标信息
                                        tar_find_dis(1,tar_ang_min) = tar_dis;
                                        tar_find_ang_rec(tar_ang_min,:,T_cnt) = [theta_pit;theta_yaw];
                                        tar_find_dis_rec(tar_ang_min,:,T_cnt) =  radar_posi + [tar_dis*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);tar_dis*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);tar_dis*sin(theta_pit*pi/180)];
                                
                                    elseif(tar_ang_sigma_min > 2 && tar_dis_sigma_min > 500)
                                        tar_find = 1 + tar_find;
                                        tar_find_ang(:,tar_find) = [theta_pit;theta_yaw];       %若与已发现目标无关，则添加为新目标
                                        tar_find_dis(1,tar_find) = tar_dis;
                                        tar_find_ang_rec(tar_find,:,T_cnt) = [theta_pit;theta_yaw];
                                        tar_find_dis_rec(tar_find,:,T_cnt) =  radar_posi + [tar_dis*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);tar_dis*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);tar_dis*sin(theta_pit*pi/180)];                       
                                    end
                                end
                            else
                                disp('noise')
                             
                            end
                        else
                            disp('nothing')
                      
                        end
                    end    
                    %figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,1)))       
                    T_cnt = T_cnt + 1;
                end
                total_data = zeros(1,3,CPI_num);                                                        %数据集合
                data_size = 1;                                                                          %用于记载当前CPI找到的目标数
                for kk = 1:tar_find
                    x_soft = zeros(3,CPI_num);
                    if(sum(sum(tar_find_dis_rec(kk,:,T_cnt - CPI_num:T_cnt))) ~= 0 )                    %判断之前发现过的目标在当前CPI有无更新的信息
                        for ii = 1:3
                            x_soft(ii,:) = soft_fil(tar_find_dis_rec(kk,ii,T_cnt - CPI_num:T_cnt-1),8); %平滑数据
                        end
                        total_data(data_size,:,:) = x_soft;                                             %若在当前CPI内信息更新了则进行跟踪
                        data_size = data_size + 1;
                    end
                end
                false_a = zeros(tar_find,3,CPI_num);                                                    %人为加入虚警点
                for kk = 1:data_size-1
                    [x1] = find(abs(total_data(kk,1,:)) == max(max(max(abs(total_data(kk,1,:))))));
                    [x2] = find(abs(total_data(kk,2,:)) == max(max(max(abs(total_data(kk,2,:))))));
                    [x3] = find(abs(total_data(kk,3,:)) == max(max(max(abs(total_data(kk,3,:))))));
                    false_a(kk,1,:) = false_a(kk,1,:) + sign(total_data(kk,1,x1(1)))*(rand(size(false_a(kk,1,:)))+0.1)*max(max(abs(total_data(kk,1,:))))*2;
                    false_a(kk,2,:) = false_a(kk,2,:) + sign(total_data(kk,2,x2(1)))*(rand(size(false_a(kk,2,:)))+0.1)*max(max(abs(total_data(kk,2,:))))*2;
                    false_a(kk,3,:) = false_a(kk,3,:) + sign(total_data(kk,3,x3(1)))*(rand(size(false_a(kk,3,:)))+0.1)*max(max(abs(total_data(kk,3,:))))*2;
                    total_data(data_size-1+kk,:,:) = false_a(kk,:,:);
                end
                figure;hold on
                for kk = 1:CPI_num
                    plot3(squeeze(total_data(:,1,kk)),squeeze(total_data(:,2,kk)),squeeze(total_data(:,3,kk)),'k.');%,'color',[rand rand rand])
                    drawnow;
                end
                xlabel('x/m');ylabel('y/m');zlabel('z/m');
                set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

                %% 跟踪滤波
                % 跟踪参数
                dis_door = 10;                                                  %距离门大小
                if_rock = zeros(1,CPI_num);                                     %是否跟踪到
                lose_num = 0;                                                   %跟丢累计数
                pre_num = 5;                                                    %预测参考数
                for tar_c = 1:data_size-1
                    tar_lose = 0;
                    tra_pro = zeros(3,CPI_num);                                 %跟踪结果
                    start_point = total_data(tar_c,:,1);                        %跟踪起点确认
                    tra_pro(:,1) = start_point;
                    for ii = 2:CPI_num
                        for kk = 1:size(total_data,1)
                            if(sum(tra_pro(:,ii-1)) == 0)
                                tra_pro(:,ii) = total_data(tar_c,:,ii);
                                continue;
                            end
                            now = total_data(kk,:,ii);                          %当前判断点
                            sigma = norm(abs(now' - tra_pro(:,ii-1)));          %算三个距离总共平方差
                            if(sigma < dis_door)
                                tra_pro(:,ii) = total_data(kk,:,ii);            %确定点
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
                            tar_lose = 1;                                       %跟丢了，继续搜索
                            disp('lose')
                            break
                        end
                    end
                    
                   % if(tar_lose == 1)
                   %     tar_find = tar_find - 1;
                   %     tar_lose_rec(tar_lose_num) = tar_c;
                   %     tar_lose_num = 1 + tar_lose_num;
                   % end
                   
                    figure;hold on
                    for kk = 1:CPI_num
                        plot3(tra_pro(1,kk),tra_pro(2,kk),tra_pro(3,kk),'k.')
                        %drawnow
                    end
                    xlabel('x/m');ylabel('y/m');zlabel('z/m');
                    hold off
                    set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

                end
                
            end
        end
end

