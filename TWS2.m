%%%%%TWS2%%%%
clear;
close all
clc
getd = @(p)path(path,p);
getd('fire_control_subpro/');
%% �ػ�ƽ̨����
Pos_Aircraft=[0;0;0]        ;   % �ػ���ά���꡾m��
V_Aircraft=[0;0;0]          ;	% �ػ��ٶ�ʸ����m/s��

%% �״�ϵͳ�������
c=3e8                       ;	% ����
k=1.38e-23                  ;	% ������������
Pt= 5000000                 ;	% ���书�ʡ�W��
Fc=10e8                     ;	% ����Ƶ�ʡ�Hz��
lambda=c/Fc                 ;	% ����������m��
Tp=1e-4                     ;	% �����ȡ�΢�롿
Fr=5*1e3                    ;	% �����ظ�Ƶ�ʡ�Hz��
B=1e6                       ;	% ����Hz��
Fs=0.01*Fc                    ;	% �����ʡ�Hz��   ���������Ǿ����й�
F=10^(5/10)                 ;	% ����ϵ��
K=B/Tp                      ;	% ��Ƶ�ʡ�Hz��
Tr=1/Fr                     ;	% �����ظ����ڡ��롿
Delta_t=1/Fs                ;	% ʱ�������ʱ�������롿
Rx_d = 0.005                ;   % ������ֱ�ӵľ��룬ע��Ͳ����ĳ��ȹ�ϵ

%% ���߲���
N_azi = 20                  ;   % ��λ��Ԫ��         %%%һ������20*20,��2������
N_pit = 20                  ;   % ������Ԫ��
ant_num = 4;
ant_pos = [Rx_d 0 Rx_d; Rx_d 0 -Rx_d; -Rx_d 0 -Rx_d; -Rx_d 0 Rx_d].';

%% Ŀ��������
% ��ʼĿ��Ƕ�  ����ĽǶ�����Էɻ����Ե�
Phi = [5 2 0 -10]./180*pi;                   %����
Sita = [-28 -3 15 -15]./180*pi;             %��λ
tar_num = 4;                                %Ŀ����
%%%%
Pos_target=[ Pos_Aircraft+ 3.50e3*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))],...             % Ŀ��1
            Pos_Aircraft+ 2e3*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...                 % Ŀ��2
            Pos_Aircraft+8e3*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))],...                  % Ŀ��3
            Pos_Aircraft+9e3*[cos(Phi(4))*sin(Sita(4)); cos(Phi(4))*cos(Sita(4)); sin(Phi(4))]];                    % Ŀ��4
%   Ŀ�����꡾m��           X������              Y������                  Z������
V_target = [ [ 2000;              3000;           5000],...     % Ŀ��2         %%%�ٶ�
    [    1000;              -5000;           3000],...          % Ŀ��2
    [    4000;              -1900;           8000],...          % Ŀ��3
    [    -2000;              2000;             2000] ];         % Ŀ��4
RCS_target=1*[1;1;1;1];                                         % �ٶ�Ŀ��RCS����������ڷ���ʱ����ں㶨
% Ŀ��ƽ������ɢ��������m^2��

%% ����������������Χ-60��+60
% ���������Ϊ5�㣬�Ͳ����������н�3�� ��λ�����5����ɨ�裬��λ��ɨ��24����
% ������������ɨ��
Beam_pit = [3  ];%-5 -10]    ;          % ����άɨ�貨��ָ���  %%%4���Ƕ�
Bean_azi = [-6:3]*5       ;                 % ��λάɨ�貨��ָ���  %%%25����λ�� -60 ���� +60
BeamShift = 2               ;               % ������������н�
CPI_num = 64               ;                % ÿ����λפ������
% ÿ��CPI128�����壬һ��ɨ��ʱ��Ϊ4*25*128*Tr=1.28s
t_set_Tp=(Delta_t:Delta_t:Tp)'          ;	% һ�������ڵ�ʱ�����������
N=length(t_set_Tp);
F_dolp = linspace(-Fr/2,Fr/2,CPI_num);      %������Ƶ��
beam_wide = 20;                             %���߷���ͼ��Χ

%% �������˲����� 
x = [0;0;0;0;0;0];                                                                          %�������˲�����״̬
x_tar = [x,x,x,x];                                                                          %�ĸ�Ŀ��
CPI_t = CPI_num*Tr;                                                                         %һ��CPI��ʱ��
F_kal = [1,0,0,Tr,0,0; 0,1,0,0,Tr,0; 0,0,1,0,0,Tr; 0,0,0,1,0,0; 0,0,0,0,1,0;0,0,0,0,0,1];   %״̬ת�ƾ���
Q_kal = eye(6,6)*0.00001;                                                                   %��������Э�������
P = eye(6,6);                                                                               %״̬Э�������
P_kal(:,:,1) = P;P_kal(:,:,2) = P;P_kal(:,:,3) = P;P_kal(:,:,4) = P;                        
R_kal = 0.1;                                                                                %�۲���������
u_kal = [0;0;0;0;0;0];                                                                      %��������
B_kal = [(Tr^2)/2;(Tr^2)/2;(Tr^2)/2;Tr;Tr;Tr];                                              %���ٶ�����


obe_val = zeros(3,tar_num);                                                                 %�۲�ֵ��¼
last_obe = zeros(3,tar_num);
meas_dis = zeros(tar_num,1);                                                                
x_record1 = zeros(6,CPI_num);                                                               %�˲������¼
x_record2 = zeros(6,CPI_num);
x_record3 = zeros(6,CPI_num);
x_record4 = zeros(6,CPI_num);
dis_rec1 = zeros(3,CPI_num);                                                                %���������¼
dis_rec2 = zeros(3,CPI_num);
dis_rec3 = zeros(3,CPI_num);
dis_rec4 = zeros(3,CPI_num);
tar_find_ang = zeros(2,1);                                                                  %��¼�ѷ���Ŀ��ĽǶ�
tar_find_dis = zeros(1,1);                                                                  %��¼�ѷ���Ŀ��ľ���
T_cnt = 1;                                                                                  %�ڼ�����������

radar_posi = Pos_Aircraft;
tar_posi = Pos_target;
tar_find = 0;                                                                               %�ѷ���Ŀ�����

while(1)
        tar_find_ang_rec = zeros(4,2,CPI_num*5*25);
        tar_find_dis_rec = zeros(4,3,CPI_num*5*25);
        disp('scan..')
        for ki = 1:length(Beam_pit)
            pit_ang = Beam_pit(ki);                                                                                         %%%������
            for km = 1:length(Bean_azi)
                tar_lose_rec = [];
                tar_lose_num = 1;                                                                                           %��¼��ʧĿ��ĸ���
                azi_ang = Bean_azi(km);                                                                                     %%%��λ��
                fprintf('Azi_Beam=%d...Pit_Beam=%d...\n',azi_ang,pit_ang);
                [beam_1,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;      % ���ɲ�������ͼ
                [beam_2,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;      % ���ɲ�������ͼ
                [beam_3,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;
                [beam_4,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;

           %     Jet_posi = Pos_Aircraft + (ki-1)*length(Bean_azi)*CPI_num*Tr*V_Aircraft + (km-1)*CPI_num*Tr*V_Aircraft  ;   % �ɻ��ڸò�λ��ʼλ��   %%%ÿɨ��һ���Ƕ�λ�ñ仯һ��  
            %    Tar_posi = Pos_target + (ki-1)*length(Bean_azi)*CPI_num*Tr*V_Aircraft + (km-1)*CPI_num*Tr*V_target      ;   % Ŀ���ڸò�λ��ʼλ��   

                CPI_sig = zeros(tar_num,round(Tr*Fs),CPI_num);                                                              %��¼����CPI�Ļز�����
                tar_dist = zeros(tar_num,CPI_num);                                                                          %Ŀ������¼
                tar_v = zeros(tar_num,CPI_num);                                                                             %Ŀ���ٶȼ�¼
                H_kal = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0]   ;                                                           %�۲����
                for kl = 1:CPI_num
                    % �ɻ�λ����Ŀ��λ�ø���
                    radar_posi = radar_posi + Tr*V_Aircraft;                                                                %ÿ�������ɻ�/Ŀ�궼���˶�һ����λ����
                    tar_posi = tar_posi + Tr*V_target;                                                                      %Ŀ��λ�ø���
                    tar_vec = zeros(ant_num,3,tar_num);                                                                     
                    for kk = 1:ant_num
                        tar_vec(kk,:,:) = tar_posi - radar_posi - ant_pos(:,kk);                                            %�ɻ���Ŀ��ľ����
                    end
                    tar_zero = tar_posi - radar_posi;                                                                       %Ŀ����ԭ��ľ���
                    % ����ϵת��
                    tar_posi_Spha = zeros(ant_num,3,tar_num);           
                    tar_posi_Spha_zero = zeros(3,tar_num);    

                    for kk = 1:tar_num
                        for ii = 1:ant_num                 
                            [r,sita,phi] = Cart2Sphe(tar_vec(ii,1,kk),tar_vec(ii,2,kk),tar_vec(ii,3,kk));                   %ת������
                            tar_posi_Spha(ii,:,kk) = [r,sita,phi];                                                          %�ĸ�Ŀ��
                        end
                        [r,sita,phi] = Cart2Sphe(tar_zero(1,kk),tar_zero(2,kk),tar_zero(3,kk));                             %ת������
                        tar_posi_Spha_zero(:,kk) = [r,sita,phi];  
                        tar_dist(kk,kl) = r;
                        if(kl > 1)
                            tar_v(kk,kl) = (tar_dist(kk,kl)-tar_dist(kk,kl-1))/Tr;                                          %��ʵ�ٶ�
                        end
                    end
                    % ���Ŀ����лز�����
                    echo = zeros(round(Tp*Fs),4);                                                                           %Tp��
                    s_echo = zeros(round(Tr*Fs),4);                                                                         %����Tr
                    s_match = zeros(round(Tr*Fs),4);
                  %% �ز�����
                    for kk = 1:tar_num
                        for ii = 1:ant_num
                            tar_dely(ii) = tar_posi_Spha(ii,1,kk)*2/c;                                                      %��Ŀ��ز���ʱ
                            Lp(ii)=1./((4*pi)^2*tar_posi_Spha(ii,1,kk).^4);                                                 %Ŀ�괦�ĵ�Ų����
                        end
                        Gt(1) = GetBeamGain(beam_1,a1,p1,tar_posi_Spha(1,2,kk),tar_posi_Spha(1,3,kk),pit_ang,azi_ang,beam_wide);   %%%�ز�����
                        Gt(2) = GetBeamGain(beam_2,a1,p1,tar_posi_Spha(2,2,kk),tar_posi_Spha(2,3,kk),pit_ang,azi_ang,beam_wide);   %%%�ز�����
                        Gt(3) = GetBeamGain(beam_3,a1,p1,tar_posi_Spha(3,2,kk),tar_posi_Spha(3,3,kk),pit_ang,azi_ang,beam_wide);   %%%�ز�����
                        Gt(4) = GetBeamGain(beam_4,a1,p1,tar_posi_Spha(4,2,kk),tar_posi_Spha(4,3,kk),pit_ang,azi_ang,beam_wide);   %%%�ز�����
                        zero_dely = tar_posi_Spha_zero(1,kk)*2/c;
                        Gr = Gt;                                                                                                   %���������������뷢������һ��
                        Echo_start=round((zero_dely)*Fs);                                                                          %Ŀ��ز���ʼ������
                        Echo_stop=Echo_start+N-1;                                                                                  %Ŀ��ز�����������                                                                           %ƥ����
                        for ii = 1:ant_num
                            Magnitude_echo(ii)=sqrt(Pt*RCS_target(kk)*(Gt(1)+Gt(2)+Gt(3)+Gt(4))*Gr(ii)*lambda^2/(4*pi)*Lp(ii));    %����A��Ŀ��ز�����
                            echo(:,ii) = echo(:,ii) + Magnitude_echo(ii)*exp(2*pi*1i*Fc*( t_set_Tp-tar_dely(ii) ) ).*exp(1i*pi*K*(t_set_Tp - tar_dely(ii)).^2);     %�ĸ����ߵĻز�
                        end
                    end
                    %% �ز�����
                    tar_record = zeros(1,4);                                                                                       %��¼�ĸ����߸��ز��Ĳ���λ��
                    Pfa = 1e-6;
                    for ii = 1:ant_num
                        s_echo(1:N,ii) = echo(:,ii);
                        s_echo(:,ii) = s_echo(:,ii) + 0*sqrt(k*B*F*290/2)*( randn(size(s_echo(:,ii) ) )+1i*randn(size(s_echo(:,ii) ) ) );           %������
                        [s_match(:,ii),R_lin,a_lin] = MatchFilter(s_echo(:,ii),Tr,Fs,Tp,Fc,K,B);
                        [XT,D,inx,iny] = my_Cfar(Pfa,s_match(:,ii),ceil(length(s_match(:,ii))/100),ceil(length(s_match(:,ii))/20));                 %cfar���
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
                    s_Sigma=zeros(round(Tr*Fs),1);                                                          % �����ͨ���ź�����
                    s_Delta_yaw=zeros(round(Tr*Fs),1);                                                      % �����ͨ���ź�����
                    s_Delta_pit=zeros(round(Tr*Fs),1);
                    for ii = 1:size(tar_record,1)
                        if(sum(tar_record(ii,:)) ~= 0)  
                            if(abs(tar_record(ii,1) - tar_record(ii,2))+abs(tar_record(ii,1)-tar_record(ii,3))+abs(tar_record(ii,1)-tar_record(ii,4))< 20)      %�鿴�������߼�⵽����Ƿ�һ��
                                s_Sigma = s_match(tar_record(ii,1),1) + s_match(tar_record(ii,2),2) + s_match(tar_record(ii,3),3) + s_match(tar_record(ii,4),4);
                                s_Delta_yaw = s_match(tar_record(ii,1),1) + s_match(tar_record(ii,2),2) - s_match(tar_record(ii,3),3) - s_match(tar_record(ii,4),4);
                                s_Delta_pit = s_match(tar_record(ii,1),1) - s_match(tar_record(ii,2),2) - s_match(tar_record(ii,3),3) + s_match(tar_record(ii,4),4);
                                %s_Sigma = s_match(:,1) + s_match(:,2) + s_match(:,3) + s_match(:,4);
                                %s_Delta_yaw = s_match(:,1) + s_match(:,2) - s_match(:,3) - s_match(:,4);
                                %s_Delta_pit = s_match(:,1) - s_match(:,2) - s_match(:,3) + s_match(:,4);
                                tar_dis = sum(tar_record(ii,:))/4/round(Tr*Fs)/2*Tr*c;                        %���
                                [in1,in2] = find(s_Sigma == max(s_Sigma));                                    %�����ֵ��Ϊ��ǵ�
                                %% ���
                                bi_yaw = imag(s_Delta_yaw(in1,in2)/s_Sigma(in1,in2));
                                bi_pit = imag(s_Delta_pit(in1,in2)/s_Sigma(in1,in2));
                                pit = atan(bi_pit)*lambda/pi/Rx_d/4;
                                theta_pit = asin(pit)*180/pi;
                                yaw = atan(bi_yaw)*lambda/pi/4/Rx_d/cos(theta_pit*pi/180);
                                theta_yaw = asin(yaw)*180/pi;

                                for tar_c = 1:tar_num
                                    real_range(tar_c) = sqrt(tar_zero(1,tar_c)^2+tar_zero(2,tar_c)^2 + tar_zero(3,tar_c)^2);
                                    real_ang(1,tar_c) = atan(tar_zero(1,tar_c)/tar_zero(2,tar_c))*180/pi;                               %��λ
                                    real_ang(2,tar_c) = atan(tar_zero(3,tar_c)/sqrt(tar_zero(2,tar_c)^2 + tar_zero(1,tar_c)^2))*180/pi; %����
                                end
                                real_ang                     ;                                                                          %��ʵ�Ƕ�;
                                %% ��¼�ѷ���Ŀ��
                                if(tar_find == 0)                                                                                       %δ����Ŀ��ʱֱ�Ӱ�Ŀ����Ϣ�������ݿ�
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
                                    %�ж��Ƿ�����·��ֵ�Ŀ��
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
                                        tar_find_ang(:,tar_ang_min) = [theta_pit;theta_yaw];    %�����ѷ��ֵ�Ŀ����أ�����¸�Ŀ����Ϣ
                                        tar_find_dis(1,tar_ang_min) = tar_dis;
                                        tar_find_ang_rec(tar_ang_min,:,T_cnt) = [theta_pit;theta_yaw];
                                        tar_find_dis_rec(tar_ang_min,:,T_cnt) =  radar_posi + [tar_dis*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);tar_dis*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);tar_dis*sin(theta_pit*pi/180)];
                                
                                    elseif(tar_ang_sigma_min > 2 && tar_dis_sigma_min > 500)
                                        tar_find = 1 + tar_find;
                                        tar_find_ang(:,tar_find) = [theta_pit;theta_yaw];       %�����ѷ���Ŀ���޹أ������Ϊ��Ŀ��
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
                total_data = zeros(1,3,CPI_num);                                                        %���ݼ���
                data_size = 1;                                                                          %���ڼ��ص�ǰCPI�ҵ���Ŀ����
                for kk = 1:tar_find
                    x_soft = zeros(3,CPI_num);
                    if(sum(sum(tar_find_dis_rec(kk,:,T_cnt - CPI_num:T_cnt))) ~= 0 )                    %�ж�֮ǰ���ֹ���Ŀ���ڵ�ǰCPI���޸��µ���Ϣ
                        for ii = 1:3
                            x_soft(ii,:) = soft_fil(tar_find_dis_rec(kk,ii,T_cnt - CPI_num:T_cnt-1),8); %ƽ������
                        end
                        total_data(data_size,:,:) = x_soft;                                             %���ڵ�ǰCPI����Ϣ����������и���
                        data_size = data_size + 1;
                    end
                end
                false_a = zeros(tar_find,3,CPI_num);                                                    %��Ϊ�����龯��
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

                %% �����˲�
                % ���ٲ���
                dis_door = 10;                                                  %�����Ŵ�С
                if_rock = zeros(1,CPI_num);                                     %�Ƿ���ٵ�
                lose_num = 0;                                                   %�����ۼ���
                pre_num = 5;                                                    %Ԥ��ο���
                for tar_c = 1:data_size-1
                    tar_lose = 0;
                    tra_pro = zeros(3,CPI_num);                                 %���ٽ��
                    start_point = total_data(tar_c,:,1);                        %�������ȷ��
                    tra_pro(:,1) = start_point;
                    for ii = 2:CPI_num
                        for kk = 1:size(total_data,1)
                            if(sum(tra_pro(:,ii-1)) == 0)
                                tra_pro(:,ii) = total_data(tar_c,:,ii);
                                continue;
                            end
                            now = total_data(kk,:,ii);                          %��ǰ�жϵ�
                            sigma = norm(abs(now' - tra_pro(:,ii-1)));          %�����������ܹ�ƽ����
                            if(sigma < dis_door)
                                tra_pro(:,ii) = total_data(kk,:,ii);            %ȷ����
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
                            tar_lose = 1;                                       %�����ˣ���������
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

