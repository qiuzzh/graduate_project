%%%%%%%%%%%%%%%%%%%%%
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
Fs=10*B                    ;	% �����ʡ�Hz��   ���������Ǿ����й�
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
Phi = [5 7 0 -8]./180*pi;                   %����
Sita = [-25 -5 -15 13]./180*pi;             %��λ
tar_num = 4;                                %Ŀ����
%%%%
Pos_target=[ Pos_Aircraft+ 3.50e3*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))],...             % Ŀ��1
            Pos_Aircraft+ 5e3*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...                 % Ŀ��2
            Pos_Aircraft+8e3*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))],...                  % Ŀ��3
            Pos_Aircraft+11e3*[cos(Phi(4))*sin(Sita(4)); cos(Phi(4))*cos(Sita(4)); sin(Phi(4))]];                   % Ŀ��4
%   Ŀ�����꡾m��           X������              Y������                  Z������
V_target = [ [  0;              200;           50],...   % Ŀ��2         %%%�ٶ�
    [    0;              -200;           30],...   % Ŀ��2
    [    0;              -190;           80],...   % Ŀ��3
    [    0;              200;             20] ];    % Ŀ��4
RCS_target=1*[1;1;1;1]; % �ٶ�Ŀ��RCS����������ڷ���ʱ����ں㶨
% Ŀ��ƽ������ɢ��������m^2��

%% ����������������Χ-60��+60
% ���������Ϊ5�㣬�Ͳ����������н�3�� ��λ�����5����ɨ�裬��λ��ɨ��24����
% ������������ɨ��
Beam_pit = [10 5 -5 -10]    ;       % ����άɨ�貨��ָ���  %%%4���Ƕ�
Bean_azi = [-6:6]*5       ;         % ��λάɨ�貨��ָ���  %%%25����λ�� -60 ���� +60
BeamShift = 2               ;       % ������������н�
CPI_num = 64               ;        % ÿ����λפ������
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
Q_kal = eye(6,6)*0.00001;                                                                      %��������Э�������
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
%%  �źŴ���

for ki = 1:4
    pit_ang = Beam_pit(ki);                                                                                         %%%������
    for km = 1:length(Bean_azi)
        tar_de_flag = zeros(tar_num,1);
        azi_ang = Bean_azi(km);                                                                                     %%%��λ��
        fprintf('Azi_Beam=%d...Pit_Beam=%d...\n',azi_ang,pit_ang);
        [beam_1,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;      % ���ɲ�������ͼ
        [beam_2,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang+BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;      % ���ɲ�������ͼ
        [beam_3,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang-BeamShift/2,beam_wide)    ;
        [beam_4,a1,p1] = Beamcreate(N_azi,N_pit,lambda,azi_ang-BeamShift/2,pit_ang+BeamShift/2,beam_wide)    ;
        
        Jet_posi = Pos_Aircraft + (ki-1)*length(Bean_azi)*CPI_num*Tr*V_Aircraft + (km-1)*CPI_num*Tr*V_Aircraft  ;   % �ɻ��ڸò�λ��ʼλ��   %%%ÿɨ��һ���Ƕ�λ�ñ仯һ��  
        Tar_posi = Pos_target + (ki-1)*length(Bean_azi)*CPI_num*Tr*V_Aircraft + (km-1)*CPI_num*Tr*V_target      ;   % Ŀ���ڸò�λ��ʼλ��   
        
        CPI_sig = zeros(tar_num,round(Tr*Fs),CPI_num);                                                              %��¼����CPI�Ļز�����
        tar_dist = zeros(tar_num,CPI_num);                                                                          %Ŀ������¼
        tar_v = zeros(tar_num,CPI_num);                                                                             %Ŀ���ٶȼ�¼
        obe_v = zeros(3,tar_num,CPI_num);
        H_kal = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0]   ;                       %�۲����
        %H_kal = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0;0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1]   ; 
        for kl = 1:CPI_num
            % �ɻ�λ����Ŀ��λ�ø���
            radar_posi = Jet_posi + kl*Tr*V_Aircraft;                                                               %ÿ�������ɻ�/Ŀ�궼���˶�һ����λ����
            tar_posi = Tar_posi + kl*Tr*V_target;                                                                   %Ŀ��λ�ø���
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
                    s_echo(:,ii) = s_echo(:,ii) + 1000*sqrt(k*B*F*290/2)*( randn(size(s_echo(:,ii) ) )+1i*randn(size(s_echo(:,ii) ) ) );           %������
                  %  figure;subplot(1,2,1);plot(linspace(0,Tr,round(Tr*Fs)),real(s_echo(:,ii)))
                   % [res] = bw_filter(s_echo(:,ii),B,Fc,Fs);
                    [s_match(:,ii),R_lin,a_lin] = MatchFilter(s_echo(:,ii),Tr,Fs,Tp,Fc,K,B);                                    %ƥ���˲�
                   % subplot(1,2,2);plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,ii)))
                end
                
                Pfa = 1e-6;
                [D1,inx1,iny1] = my_Cfar(Pfa,s_match(:,1),ceil(length(s_match(:,1))/100),ceil(length(s_match(:,1))/100));     %cfar���
                [D2,inx2,iny2] = my_Cfar(Pfa,s_match(:,2),ceil(length(s_match(:,2))/100),ceil(length(s_match(:,2))/100));
                [D3,inx3,iny3] = my_Cfar(Pfa,s_match(:,3),ceil(length(s_match(:,3))/100),ceil(length(s_match(:,3))/100));
                [D4,inx4,iny4] = my_Cfar(Pfa,s_match(:,4),ceil(length(s_match(:,4))/100),ceil(length(s_match(:,4))/100));
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

                        for ii = 1:tar_num
                            real_range(ii) = sqrt(tar_zero(1,ii)^2+tar_zero(2,ii)^2 + tar_zero(3,ii)^2);
                            real_ang(1,ii) = atan(tar_zero(1,ii)/tar_zero(2,ii))*180/pi;                            %��λ
                            real_ang(2,ii) = atan(tar_zero(3,ii)/sqrt(tar_zero(2,ii)^2 + tar_zero(1,ii)^2))*180/pi; %����
                        end
                        real_ang    %��ʵ�Ƕ�;
                    %% �������˲�
                        CPI_sig(kk,:,kl) = s_match(:,1);
                        last_obe(:,kk) = obe_val(:,kk);
                        obe_val(:,kk) = radar_posi + [meas_dis(kk,1)*cos(theta_pit*pi/180)*sin(theta_yaw*pi/180);meas_dis(kk,1)*cos(theta_pit*pi/180)*cos(theta_yaw*pi/180);meas_dis(kk,1)*sin(theta_pit*pi/180)]; %����۲�ֵ
                        if(kl>1)
                            obe_v(:,kk,kl) = (obe_val(:,kk) - last_obe(:,kk))/Tr;
                        else
                            obe_v(:,kk,kl) = 0;
                        end
                        kal_obe = [obe_val(:,kk)];
                        [x_tar(:,kk),P_kal(:,:,kk)] = my_kalman_filter(x_tar(:,kk),P_kal(:,:,kk),H_kal,F_kal,R_kal,Q_kal,u_kal,B_kal,kal_obe);
                        x_record1(:,kl) = x_tar(:,1);       %��¼Ŀ���6�����ĸ��ٽ��
                        x_record2(:,kl) = x_tar(:,2);
                        x_record3(:,kl) = x_tar(:,3);
                        x_record4(:,kl) = x_tar(:,4);
                        dis_rec1(:,kl) = obe_val(:,1);      %��¼�۲�ֵ
                        dis_rec2(:,kl) = obe_val(:,2);
                        dis_rec3(:,kl) = obe_val(:,3);
                        dis_rec4(:,kl) = obe_val(:,4);
                    else
                      % disp('noise')
                        tar_de_flag(kk,1) = tar_de_flag(kk,1) + 1;
                        CPI_sig(kk,:,kl) = 0;
                    end
                else
                    %disp('nothing')
                   tar_de_flag(kk,1) = tar_de_flag(kk,1) + 1;
                   CPI_sig(kk,:,kl) = 0;
                end
                
                %figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(s_match(:,1)))
                
            end
            
            
        end
        %% mtd����
        ff = linspace(-Fr/2,Fr/2,CPI_num);                                      %������Ƶ�׷�Χ
        RD_sig = zeros(tar_num,CPI_num);                                        %�洢��ʱ��fft
        s_max = zeros(tar_num,1);
        for ii = 1:tar_num
            if(tar_de_flag(ii,1) < CPI_num/2)
                [s_max] = find(CPI_sig(ii,:,1) == max(CPI_sig(ii,:,1)));        %�Ҿ����ϵ����ֵ
                figure;plot(linspace(0,Tr,round(Tr*Fs)),abs(CPI_sig(ii,:,1)))
                RD_sig(ii,:) = fftshift(fft(CPI_sig(ii,s_max,:)));              %�Ծ��������ֵ��fft
                figure;plot(ff,abs(RD_sig(ii,:)))
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
       
    end
end





