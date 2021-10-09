%%%%%STT%%%%%%%%%%%%%%%%
function [tar_find_ang,tar_find_dis,tar_find_v,tar_posi,radar_posi,real_ang,tar_rock,vd] = Stt_func(Pt,Fc,Tp,Tr,Fs,Pos_target,V_target,RCS_target,B,Pos_Aircraft,V_Aircraft,Phi,Sita,tar_rock,tar_ang)
clc
getd = @(p)path(path,p);
getd('fire_control_subpro/');


%% �״�ϵͳ�������
c=3e8                       ;	% ����
k=1.38e-23                  ;	% ������������
%Pt= 5000000                  ;	% ���书�ʡ�W��
%Fc=10e8                     ;	% ����Ƶ�ʡ�Hz��
lambda=c/Fc                 ;	% ����������m��
%Tp=1e-4                    ;	% �����ȡ�΢�롿
Fr=1/Tr                     ;	% �����ظ�Ƶ�ʡ�Hz��
%B=1e6                       ;	% ����Hz��
%Fs=0.1*Fc                      ;	% �����ʡ�Hz��
F=10^(5/10)                 ;	% ����ϵ��
K=B/Tp                      ;	% ��Ƶ�ʡ�Hz��
%Tr=1/Fr                     ;	% �����ظ����ڡ��롿
Delta_t=1/Fs                ;	% ʱ�������ʱ�������롿
Rx_d = 0.03;

%% ���߲���
N_azi = 20                  ;   % ��λ��Ԫ��         %%%һ������20*20,��2������
N_pit = 20                  ;   % ������Ԫ��
ant_num = 4;
ant_pos = [Rx_d 0 Rx_d; Rx_d 0 -Rx_d; -Rx_d 0 -Rx_d; -Rx_d 0 Rx_d].';

%% Ŀ��������
% ��ʼĿ��Ƕ�  ����ĽǶ�����Էɻ����Ե�
%Phi = [5 ]./180*pi;                   %����
%Sita = [-10 ]./180*pi;             %��λ
tar_num = length(Phi);                                %Ŀ����
%{
%%%%
Pos_target=[ Pos_Aircraft+ 8e3*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))]];                   % Ŀ��4
%   Ŀ�����꡾m��           X������              Y������                  Z������
V_target = [ [  1000;              2000;           5000]  ];    % Ŀ��4
RCS_target=1*[1]; % �ٶ�Ŀ��RCS����������ڷ���ʱ����ں㶨
% Ŀ��ƽ������ɢ��������m^2��
%}
tra_lose = 0;

%% ����������������Χ-60��+60
% ���������Ϊ5�㣬�Ͳ����������н�3�� ��λ�����5����ɨ�裬��λ��ɨ��24����
% ������������ɨ��
Beam_pit = [10 5 0 -5 -10]    ;       % ����άɨ�貨��ָ���  %%%4���Ƕ�
Bean_azi = [-6:6]*5       ;         % ��λάɨ�貨��ָ���  %%%25����λ�� -60 ���� +60
BeamShift = 2               ;       % ������������н�
TRA_CPI = 128;
CPI_num = 1               ;        % ÿ����λפ������
% ÿ��CPI128�����壬һ��ɨ��ʱ��Ϊ4*25*128*Tr=1.28s
t_set_Tp=(Delta_t:Delta_t:Tp)'          ;	% һ�������ڵ�ʱ�����������
N=length(t_set_Tp);
%F_dolp = linspace(-Fr/2,Fr/2,CPI_num);      %������Ƶ��
beam_wide = 20;                             %���߷���ͼ��Χ

%% �������˲�����
x = [0;0;0;0;0;0];                                                                          %�������˲�����״̬
x_tar = [x];                                                                          %�ĸ�Ŀ��
CPI_t = CPI_num*Tr;                                                                         %һ��CPI��ʱ��
F_kal = [1,0,0,Tr,0,0; 0,1,0,0,Tr,0; 0,0,1,0,0,Tr; 0,0,0,1,0,0; 0,0,0,0,1,0;0,0,0,0,0,1];   %״̬ת�ƾ���
Q_kal = eye(6,6)*0.05;                                                                      %��������Э�������
P = eye(6,6);                                                                               %״̬Э�������
P_kal(:,:,1) = P;
R_kal = 0.1;                                                                                %�۲���������
u_kal = [0;0;0;0;0;0];                                                                      %��������
B_kal = [(Tr^2)/2;(Tr^2)/2;(Tr^2)/2;Tr;Tr;Tr];                                              %���ٶ�����
obe_val = zeros(3,1);                                                                 %�۲�ֵ��¼
meas_dis = zeros(1,TRA_CPI);
CPI_N = 1;                                                                                  %������
CPI_sum = 0;

radar_posi = Pos_Aircraft;
tar_posi = Pos_target;
tar_find_ang = zeros(2,1);
tar_find_dis = zeros(4,1);
tar_find_v = zeros(3,1);
%%  �źŴ���
if(tar_rock == 0)
    disp('scan..')
    tar_lose = 0;
    for ki = 1:length(Beam_pit)
        pit_ang = Beam_pit(ki);                                                                                         %%%������
        for km = 1:length(Bean_azi)
            tar_de_flag = zeros(tar_num,1);
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
                radar_posi = radar_posi + Tr*V_Aircraft;                                                               %ÿ�������ɻ�/Ŀ�궼���˶�һ����λ����
                tar_posi = tar_posi + Tr*V_target;                                                                   %Ŀ��λ�ø���
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
                    Gr = Gt;
                    Echo_start=round((zero_dely)*Fs);                                                                          %Ŀ��ز���ʼ������
                    Echo_stop=Echo_start+N-1;                                                                                  %Ŀ��ز�����������
                    echo = zeros(round(Tp*Fs),4);                                                                              %Tp��
                    s_echo = zeros(round(Tr*Fs),4);                                                                            %����Tr
                    s_match = zeros(round(Tr*Fs),4);                                                                           %ƥ����
                    for ii = 1:ant_num
                        Magnitude_echo(ii)=sqrt(Pt*RCS_target(kk)*(Gt(1)+Gt(2)+Gt(3)+Gt(4))*Gr(ii)*lambda^2/(4*pi)*Lp(ii));    %����A��Ŀ��ز�����
                        echo(:,ii) = Magnitude_echo(ii)*exp(2*pi*1i*Fc*( t_set_Tp-tar_dely(ii) ) ).*exp(1i*pi*K*(t_set_Tp - tar_dely(ii)).^2);         %�ĸ����ߵĻز�
                        s_echo(1:N,ii) = echo(:,ii);
                        % figure;plot(linspace(0,Tr,round(Tr*Fs)),real(s_echo(:,ii)))
                        s_echo(:,ii) = s_echo(:,ii) + 0*sqrt(k*B*F*290/2)*( randn(size(s_echo(:,ii) ) )+1i*randn(size(s_echo(:,ii) ) ) );           %������
                        %figure;subplot(1,2,1);plot(linspace(0,Tr,round(Tr*Fs)),real(s_echo(:,ii)))
                        % [res] = bw_filter(s_echo(:,ii),B,Fc,Fs);
                        [s_match(:,ii),R_lin,a_lin] = MatchFilter(s_echo(:,ii),Tr,Fs,Tp,Fc,K,B);                                    %ƥ���˲�
                        %subplot(1,2,2);plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,ii)))
                    end
                    
                    Pfa = 1e-6;
                    [XT1,D1,inx1,iny1] = my_Cfar(Pfa,s_match(:,1),ceil(length(s_match(:,1))/200),ceil(length(s_match(:,1))/20));     %cfar���
                    [XT2,D2,inx2,iny2] = my_Cfar(Pfa,s_match(:,2),ceil(length(s_match(:,2))/200),ceil(length(s_match(:,2))/20));
                    [XT3,D3,inx3,iny3] = my_Cfar(Pfa,s_match(:,3),ceil(length(s_match(:,3))/200),ceil(length(s_match(:,3))/20));
                    [XT4,D4,inx4,iny4] = my_Cfar(Pfa,s_match(:,4),ceil(length(s_match(:,4))/200),ceil(length(s_match(:,4))/20));
                    % inx4 = inx1;inx2 = inx1;inx3 = inx1;iny4 = iny1;iny2 = iny1;iny3 = iny1;
                    s_Sigma=zeros(round(Tr*Fs),1);                                              % �����ͨ���ź�����
                    s_Delta_yaw=zeros(round(Tr*Fs),1);                                          % �����ͨ���ź�����
                    s_Delta_pit=zeros(round(Tr*Fs),1);
                    if( ~isempty(inx1) && ~isempty(inx2) && ~isempty(inx3) && ~isempty(inx4))   %������ֵ
                        [max_x1,max_y1] = find(s_match == max(s_match(iny1,1)));
                        [max_x2,max_y2] = find(s_match == max(s_match(iny2,1)));
                        [max_x3,max_y3] = find(s_match == max(s_match(iny3,1)));
                        [max_x4,max_y4] = find(s_match == max(s_match(iny4,1)));
                        
                        if(abs(max_x1 - max_x2)+abs(max_x1-max_x3)+abs(max_x1-max_x4)< 20)      %�鿴�������߼�⵽����Ƿ�һ��
                            s_Sigma = s_match(max_x1,1) + s_match(max_x2,2) + s_match(max_x3,3) + s_match(max_x4,4);
                            s_Delta_yaw = s_match(max_x1,1) + s_match(max_x2,2) - s_match(max_x3,3) - s_match(max_x4,4);
                            s_Delta_pit = s_match(max_x1,1) - s_match(max_x2,2) - s_match(max_x3,3) + s_match(max_x4,4);
                            %s_Sigma = s_match(:,1) + s_match(:,2) + s_match(:,3) + s_match(:,4);
                            %s_Delta_yaw = s_match(:,1) + s_match(:,2) - s_match(:,3) - s_match(:,4);
                            %s_Delta_pit = s_match(:,1) - s_match(:,2) - s_match(:,3) + s_match(:,4);
                            meas_dis(kk,1) = max_x1/round(Tr*Fs)/2*Tr*c;                        %���
                            [in1,in2] = find(s_Sigma == max(s_Sigma));                          %�����ֵ��Ϊ��ǵ�
                            %% ���
                            bi_yaw = imag(s_Delta_yaw(in1,in2)/s_Sigma(in1,in2));
                            bi_pit = imag(s_Delta_pit(in1,in2)/s_Sigma(in1,in2));
                            pit = atan(bi_pit)*lambda/pi/Rx_d/4;
                            theta_pit = asin(pit)*180/pi
                            yaw = atan(bi_yaw)*lambda/pi/4/Rx_d/cos(theta_pit*pi/180);
                            theta_yaw = asin(yaw)*180/pi
                            tar_find_ang = [theta_pit;theta_yaw];
                            tar_find_dis = [meas_dis(kk,1);radar_posi + [meas_dis(kk,1)*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);meas_dis(kk,1)*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);meas_dis(kk,1)*sin(theta_pit*pi/180)]];
                            for ii = 1:tar_num
                                real_range(ii) = sqrt(tar_zero(1,ii)^2+tar_zero(2,ii)^2 + tar_zero(3,ii)^2);
                                real_ang(1,ii) = atan(tar_zero(1,ii)/tar_zero(2,ii))*180/pi;                            %��λ
                                real_ang(2,ii) = atan(tar_zero(3,ii)/sqrt(tar_zero(2,ii)^2 + tar_zero(1,ii)^2))*180/pi; %����
                            end
                            real_ang    %��ʵ�Ƕ�;
                            %% �������˲�
                            CPI_sig(kk,:,kl) = s_match(:,1);
                            obe_val(:,kk) = radar_posi + [meas_dis(kk,1)*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);meas_dis(kk,1)*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);meas_dis(kk,1)*sin(theta_pit*pi/180)]; %����۲�ֵ
                            %   [x_tar(:,kk),P_kal(:,:,kk)] = my_kalman_filter(x_tar(:,kk),P_kal(:,:,kk),H_kal,F_kal,R_kal,Q_kal,u_kal,B_kal,obe_val(:,kk));
                            %  x_record1(:,kl) = x_tar(:,1);       %��¼Ŀ���6�����ĸ��ٽ��
                            % x_record2(:,kl) = x_tar(:,2);
                            %x_record3(:,kl) = x_tar(:,3);
                            %x_record4(:,kl) = x_tar(:,4);
                            %dis_rec1(:,kl) = obe_val(:,1);      %��¼�۲�ֵ
                            %dis_rec2(:,kl) = obe_val(:,2);
                            %dis_rec3(:,kl) = obe_val(:,3);
                            %dis_rec4(:,kl) = obe_val(:,4);
                            %% �������״̬
                            tar_rock = 1;
                            now_pit = Beam_pit(ki);
                            now_yaw = Bean_azi(km);
                            last_obe = obe_val(:,kk);
                            break;
                        else
                            disp('noise')
                            tar_de_flag(kk,1) = tar_de_flag(kk,1) + 1;
                            CPI_sig(kk,:,kl) = 0;
                        end
                    else
                        disp('nothing')
                        tar_de_flag(kk,1) = tar_de_flag(kk,1) + 1;
                        CPI_sig(kk,:,kl) = 0;
                    end
                    
                    %figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,1)))
                    
                end
                
                
                
            end
            %{
                %% mtd����
                ff = linspace(-Fr/2,Fr/2,CPI_num);                                      %������Ƶ�׷�Χ
                RD_sig = zeros(tar_num,CPI_num);                                        %�洢��ʱ��fft
                s_max = zeros(tar_num,1);
                for ii = 1:tar_num
                    if(tar_de_flag(ii,1) < CPI_num/2)
                        [s_max] = find(CPI_sig(ii,:,1) == max(CPI_sig(ii,:,1)));        %�Ҿ����ϵ����ֵ
                   %     figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(CPI_sig(ii,:,1)))
                        RD_sig(ii,:) = fftshift(fft(CPI_sig(ii,s_max,:)));              %�Ծ��������ֵ��fft
                        %figure;plot(ff,abs(RD_sig(ii,:)))
                        [ff_max] = find(abs(RD_sig(ii,:)) == max(abs(RD_sig(ii,:))));   %��Ƶ�����ֵ
                        fd = ff(ff_max);
                        tar_v(ii,1);                                                    %��ʵ�ٶ�
                        vd = -fd*lambda/2;                                              %���ٽ��
                        %H_kal = eye(6,6);
                       % obe_val2 = [obe_val(1,kk);obe_val(2,kk);obe_val(3,kk);]
                      %  [x_tar(:,kk),P_tar(:,:,kk)] = my_kalman_filter(x_tar(:,kk),P_tar(:,:,kk),H_kal,F_kal,R_kal,Q_kal,u_kal,B_kal,obe_val(:,kk));
                        %x = my_kalman_filter()
                    end
                end
            %}
            if(tar_rock == 1)
                break
            end
            
        end
        if(tar_rock == 1)
            break
        end
    end
    
else
    %% ����
    x_record1 = zeros(6,TRA_CPI*CPI_N);                                                               %�˲������¼
    dis_rec1 = zeros(4,TRA_CPI*CPI_N);                                                                %���������¼
    real_dis = zeros(3,3,TRA_CPI*CPI_N);
    ang = zeros(2,TRA_CPI*CPI_N);
    tar_de_flag = zeros(1,1);
    tar_lose = 0;
    for T = 1:CPI_N
        disp('tracing...')
        pit_ang = tar_ang(1);
        azi_ang = tar_ang(2);
        [beam_1,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;      % ���ɲ�������ͼ
        [beam_2,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;      % ���ɲ�������ͼ
        [beam_3,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;
        [beam_4,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;
        CPI_sig = zeros(round(Tr*Fs),TRA_CPI);                                                              %��¼����CPI�Ļز�����
        tar_dist = zeros(tar_num,TRA_CPI);                                                                          %Ŀ������¼
        tar_v = zeros(tar_num,TRA_CPI);                                                                             %Ŀ���ٶȼ�¼
        obe_v = zeros(3,CPI_num);
        H_kal = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0]   ;                                                           %�۲����
        for kl = 1:TRA_CPI
            echo = zeros(round(Tp*Fs),4);                                                                              %Tp��
            s_echo = zeros(round(Tr*Fs),4);                                                                            %����Tr
            s_match = zeros(round(Tr*Fs),4);
            % �ɻ�λ����Ŀ��λ�ø���
            radar_posi = radar_posi + Tr*V_Aircraft;                                                               %ÿ�������ɻ�/Ŀ�궼���˶�һ����λ����
            tar_posi = tar_posi + Tr*V_target;                                                                   %Ŀ��λ�ø���
            tar_vec = zeros(ant_num,3,tar_num);
            % real_dis(:,kl+CPI_sum*TRA_CPI) = tar_posi;
            real_dis(:,:,kl) = tar_posi;
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
                Gr = Gt;
                Echo_start=round((zero_dely)*Fs);                                                                          %Ŀ��ز���ʼ������
                Echo_stop=Echo_start+N-1;                                                                                  %Ŀ��ز�����������                                                                           %ƥ����
                for ii = 1:ant_num
                    Magnitude_echo(ii)=sqrt(Pt*RCS_target(kk)*(Gt(1)+Gt(2)+Gt(3)+Gt(4))*Gr(ii)*lambda^2/(4*pi)*Lp(ii));    %����A��Ŀ��ز�����
                    echo(:,ii) = echo(:,ii) + Magnitude_echo(ii)*exp(2*pi*1i*Fc*( t_set_Tp-tar_dely(ii) ) ).*exp(1i*pi*K*(t_set_Tp - tar_dely(ii)).^2);         %�ĸ����ߵĻز�
                end
            end
            for ii = 1:ant_num
                    s_echo(1:N,ii) = echo(:,ii);
                    s_echo(:,ii) = s_echo(:,ii) + 0*sqrt(k*B*F*290/2)*( randn(size(s_echo(:,ii) ) )+1i*randn(size(s_echo(:,ii) ) ) );           %������
                    [s_match(:,ii),R_lin,a_lin] = MatchFilter(s_echo(:,ii),Tr,Fs,Tp,Fc,K,B);
                    %       [D,inx,iny] = my_Cfar(Pfa,s_match(:,ii),ceil(length(s_match(:,ii))/100),ceil(length(s_match(:,ii))/80));     %cfar���
                     %figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,ii)))                   
            end
            Pfa = 1e-6;
            [XT1,D1,inx1,iny1] = my_Cfar(Pfa,s_match(:,1),ceil(length(s_match(:,1))/200),ceil(length(s_match(:,1))/20));     %cfar���
            [XT2,D2,inx2,iny2] = my_Cfar(Pfa,s_match(:,2),ceil(length(s_match(:,2))/200),ceil(length(s_match(:,2))/20));
            [XT3,D3,inx3,iny3] = my_Cfar(Pfa,s_match(:,3),ceil(length(s_match(:,3))/200),ceil(length(s_match(:,3))/20));
            [XT4,D4,inx4,iny4] = my_Cfar(Pfa,s_match(:,4),ceil(length(s_match(:,4))/200),ceil(length(s_match(:,4))/20));
            % inx4 = inx1;inx2 = inx1;inx3 = inx1;iny4 = iny1;iny2 = iny1;iny3 = iny1;
            s_Sigma=zeros(round(Tr*Fs),1);                                              % �����ͨ���ź�����
            s_Delta_yaw=zeros(round(Tr*Fs),1);                                          % �����ͨ���ź�����
            s_Delta_pit=zeros(round(Tr*Fs),1);
            if( ~isempty(inx1) && ~isempty(inx2) && ~isempty(inx3) && ~isempty(inx4))   %������ֵ
                [max_x1,max_y1] = find(s_match == max(s_match(iny1,1)));
                [max_x2,max_y2] = find(s_match == max(s_match(iny2,1)));
                [max_x3,max_y3] = find(s_match == max(s_match(iny3,1)));
                [max_x4,max_y4] = find(s_match == max(s_match(iny4,1)));
                
                if(abs(max_x1 - max_x2)+abs(max_x1-max_x3)+abs(max_x1-max_x4)< 20)      %�鿴�������߼�⵽����Ƿ�һ��
                    s_Sigma = s_match(max_x1,1) + s_match(max_x2,2) + s_match(max_x3,3) + s_match(max_x4,4);
                    s_Delta_yaw = s_match(max_x1,1) + s_match(max_x2,2) - s_match(max_x3,3) - s_match(max_x4,4);
                    s_Delta_pit = s_match(max_x1,1) - s_match(max_x2,2) - s_match(max_x3,3) + s_match(max_x4,4);
                    %s_Sigma = s_match(:,1) + s_match(:,2) + s_match(:,3) + s_match(:,4);
                    %s_Delta_yaw = s_match(:,1) + s_match(:,2) - s_match(:,3) - s_match(:,4);
                    %s_Delta_pit = s_match(:,1) - s_match(:,2) - s_match(:,3) + s_match(:,4);
                    meas_dis(1,kl) = (max_x1+max_x2+max_x3+max_x4)/4/round(Tr*Fs)/2*Tr*c;                        %���
                    [in1,in2] = find(s_Sigma == max(s_Sigma));                          %�����ֵ��Ϊ��ǵ�
                    %% ���
                    bi_yaw = imag(s_Delta_yaw(in1,in2)/s_Sigma(in1,in2));
                    bi_pit = imag(s_Delta_pit(in1,in2)/s_Sigma(in1,in2));
                    pit = atan(bi_pit)*lambda/pi/Rx_d/4;
                    theta_pit = asin(pit)*180/pi
                    yaw = atan(bi_yaw)*lambda/pi/4/Rx_d/cos(theta_pit*pi/180);
                    theta_yaw = asin(yaw)*180/pi
                    %   tar_find_ang = [theta_pit;theta_yaw];
                    % ang(:,kl+CPI_sum*TRA_CPI) = [theta_pit;theta_yaw];
                    ang(:,kl) = [theta_pit;theta_yaw];
                    for ii = 1:tar_num
                        real_range(ii) = sqrt(tar_zero(1,ii)^2+tar_zero(2,ii)^2 + tar_zero(3,ii)^2);
                        real_ang(1,ii) = atan(tar_zero(1,ii)/tar_zero(2,ii))*180/pi;                            %��λ
                        real_ang(2,ii) = atan(tar_zero(3,ii)/sqrt(tar_zero(2,ii)^2 + tar_zero(1,ii)^2))*180/pi; %����
                    end
                    real_ang    %��ʵ�Ƕ�
                    %% �������˲�
                    CPI_sig(:,kl) = s_match(:,1);
                    last_obe(:,1) = obe_val(:,1);
                    obe_val(:,1) = radar_posi + [meas_dis(1,kl)*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);meas_dis(1,kl)*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);meas_dis(1,kl)*sin(theta_pit*pi/180)]; %����۲�ֵ
                    %tar_find_dis = [meas_dis(kk,1);radar_posi + [meas_dis(kk,1)*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);meas_dis(kk,1)*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);meas_dis(kk,1)*sin(theta_pit*pi/180)]];
                     kal_obe = [obe_val(:,1)];
                    [x_tar(:,1),P_kal(:,:,1)] = my_kalman_filter(x_tar(:,1),P_kal(:,:,1),H_kal,F_kal,R_kal,Q_kal,u_kal,B_kal,kal_obe);
                    % x_record1(:,kl+CPI_sum*TRA_CPI) = x_tar(:,1);       %��¼Ŀ���6�����ĸ��ٽ��
                    % dis_rec1(:,kl+CPI_sum*TRA_CPI) = obe_val(:,1);      %��¼�۲�ֵ
                    x_record1(:,kl) = x_tar(:,1);
                    dis_rec1(:,kl) = [meas_dis(1,kl);obe_val(:,1)];
                else
                    disp('noise')
                    tar_de_flag(1,1) = tar_de_flag(1,1) + 1;
                    CPI_sig(:,kl) = 0;
                end
            else
                disp('nothing')
                tar_de_flag(1,1) = tar_de_flag(1,1) + 1;
                CPI_sig(:,kl) = 0;
                
            end
            
            
        end
        %{
                figure; hold on
                for ki = 1:TRA_CPI
                    plot3(x_record1(1,ki),x_record1(2,ki),x_record1(3,ki),'.')
                    drawnow;
                    %     pause(0.01)
                end
        %}
        CPI_sum = CPI_sum + 1;
    end
    x_soft = zeros(3,TRA_CPI*CPI_N);
    for kk = 1:3
        x_soft(kk,:) = soft_fil(dis_rec1(kk,:),8);
    end
    %�����龯��
    false_a = zeros(3,3,TRA_CPI*CPI_N);
    for kk = 1:3
        false_a(1,kk,:) = false_a(1,kk,:) + (rand(size(false_a(1,kk,:)))+0.1)*max(max(max(x_soft(1,:))))*2;
        false_a(2,kk,:) = false_a(2,kk,:) + (rand(size(false_a(2,kk,:)))+0.1)*max(max(max(x_soft(2,:))))*2;
        false_a(3,kk,:) = false_a(3,kk,:) + (rand(size(false_a(3,kk,:)))+0.1)*max(max(max(x_soft(3,:))))*2;
    end
    total_data(:,1,:) = x_soft;
    total_data(:,2,:) = false_a(:,1,:);
    total_data(:,3,:) = false_a(:,2,:);
    total_data(:,4,:) = false_a(:,3,:);
    figure;hold on
    for kk = 1:TRA_CPI*CPI_N
        plot3(squeeze(total_data(1,:,kk)),squeeze(total_data(2,:,kk)),squeeze(total_data(3,:,kk)),'.','color',[rand rand rand])
        drawnow;
    end
    xlabel('x');ylabel('y');zlabel('z');
    %% mtd����
    ff = linspace(-Fr/2,Fr/2,TRA_CPI);                                      %������Ƶ�׷�Χ
    RD_sig = zeros(1,TRA_CPI);                                        %�洢��ʱ��fft
    s_max = zeros(tar_num,1);
    for ii = 1:1
        if(tar_de_flag(ii,1) < TRA_CPI/2)
            [s_max] = find(CPI_sig(:,1) == max(CPI_sig(:,1)));        %�Ҿ����ϵ����ֵ
            %     figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(CPI_sig(ii,:,1)))
            RD_sig(1,:) = fftshift(fft(CPI_sig(s_max(1),:)));              %�Ծ��������ֵ��fft
            %figure;plot(ff,abs(RD_sig(ii,:)))
            [ff_max] = find(abs(RD_sig(1,:)) == max(abs(RD_sig(1,:))));   %��Ƶ�����ֵ
            fd = ff(ff_max);
            tar_v(ii,1);                                                    %��ʵ�ٶ�
            vd = -fd*lambda/2;                                              %���ٽ��
            %H_kal = eye(6,6);
            % obe_val2 = [obe_val(1,kk);obe_val(2,kk);obe_val(3,kk);]
            %  [x_tar(:,kk),P_tar(:,:,kk)] = my_kalman_filter(x_tar(:,kk),P_tar(:,:,kk),H_kal,F_kal,R_kal,Q_kal,u_kal,B_kal,obe_val(:,kk));
            %x = my_kalman_filter()
        end
    end
    
    %% �����˲�
    % ���ٲ���
    dis_door = 8;                           %�����Ŵ�С
    if_rock = zeros(1,TRA_CPI*CPI_N);       %�Ƿ���ٵ�
    lose_num = 0;                           %�����ۼ���
    pre_num = 5;                           %Ԥ��ο���
    tra_pro = zeros(3,TRA_CPI*CPI_N);       %���ٽ��
    start_point = total_data(:,1,1);        %�������ȷ��
    tra_pro(:,1) = start_point;
    for ii = 2:TRA_CPI*CPI_N
        for kk = 1:3
            now = total_data(:,kk,ii);                  %��ǰ�жϵ�
            sigma = norm(abs(now - tra_pro(:,ii-1)));        %�����������ܹ�ƽ����
            if(sigma < dis_door)
                tra_pro(:,ii) = total_data(:,kk,ii);    %ȷ����
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
            tar_lose = 1;                               %�����ˣ���������
            disp('lose')
            break
        end
    end
    
    total_dis_cha = [0;0;0];
    for kk = 2:TRA_CPI*CPI_N
        total_dis_cha = tra_pro(:,kk) - tra_pro(:,kk-1) + total_dis_cha;
    end
    tar_find_v(:,1) = sum(total_dis_cha,2)/(TRA_CPI*CPI_N-1)/Tr;
    
    if(tar_lose == 1)
        tar_rock = 0;
        tar_find_ang = ang(:,ii-2);
        tar_find_dis = dis_rec1(:,ii-2);
    else
        tar_find_ang = ang(:,end);
        tar_find_dis = dis_rec1(:,end);
    end
    figure;hold on
    for kk = 1:TRA_CPI*CPI_N
        plot3(tra_pro(1,kk),tra_pro(2,kk),tra_pro(3,kk),'.')
        %drawnow
    end
    xlabel('x');ylabel('y');zlabel('z');
    hold off
end




