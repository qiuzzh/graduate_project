%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           雷达PRF选择
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;close all;clc
getd = @(p)path(path,p);
getd('fire_control_subpro/');
c = 3e8;
fc =10e9;
lambda = c/fc;
B = 6*1e6;
fs = 0.001*fc;

%% PRF  LPRF:1K~3K   MPRF:10k~20K   HPRF:200K_250K
prf = [1e3];

%%  最大不模糊距离以及最大不模糊速度
pri = 1./prf;                %发射周期  
nrn = round(pri*fs/2)*2;    %采样点数
Te = pri;
K = B./Te;
R_maxUnAmbi = c/2.*pri;      %最大不模糊距离
V_maxUnambi = prf.*lambda/4; %最大不模糊速度

%% 目标参数
tar_r = [5e4,5e3];      %目标距离
tar_v = [-200,10];        %目标速度
dop_f = 2*tar_v/lambda;
tar_num = length(tar_r);
%% 回波仿真

nan =256*1;                %周期数
mohu = length(prf);
na_lin = pri.*(0:nan-1);     %整个过程时间


%Kr = 4*pi*(fr+fc)/c;                    %
total_t = zeros(mohu,tar_num);
total_dis = zeros(mohu,tar_num);
for kk = 1:mohu
    tar_trace = tar_r + tar_v.*na_lin(kk,:)';    %目标位置变化
    echo = zeros(nrn(kk),nan);
    match_sig = zeros(nrn(kk),nan);
    fr = [-nrn(kk)/2:nrn(kk)/2-1]./nrn(kk)*fs;          %频谱
    t = linspace(0,Te(kk),nrn(kk));
    for ki = 1:nan
        for km = 1:tar_num
           p_r =tar_trace(ki,km);
           % sig = exp(-1i*Kr*p_r);
           dely = 2*p_r/c;
           sig = exp(1i*2*pi*fc*(t-dely)).*exp(1i*K*pi*(t-dely).^2);
           echo(:,ki) = echo(:,ki) + sig';
           %{
           sig2 = exp(1i*2*pi*fc*(t)).*exp(1i*K*pi*(t).^2);
           e_sig = conj(fftshift(fft(sig2))); 
           F1 = fftshift(fft(sig));
           F = e_sig.*F1;
           s = ifft(fftshift(F));
           figure;plot(t,abs(s));
          % plot(fr,abs(fftshift(fft(echo(:,ki)))))
           %}
        end
    end

    Delt_R = c/2/fs;                        %每个采样点间隔距离
    r_lin = (0:nrn(kk)-1)*Delt_R;               %总距离
    fd_lin = linspace(-prf(kk)/2,prf(kk)/2,nan);    %多普勒不模糊范围%

    %com_sig = ifft(echo);

    e_sig = conj(fftshift(fft(exp(2*pi*fc*t*1i).*exp(1i*pi*K(kk)*t.^2) ))); 
   
    %F = e_sig.*fftshift(fft(sig));
    %s = ifft(fftshift(F));
    %figure;plot(t,abs(s));
    
    
    
    for ki = 1:nan
        F1 = fftshift(fft(echo(:,ki)'));
        %plot(fr,abs(F1))
        F = e_sig.*F1;
        match_sig(:,ki) = ifft(fftshift(F))';
      %  figure;plot(t,match_sig(:,ki))
    end
    
    match_sig1 = (max(max(abs(match_sig))) - abs(match_sig))/max(max(abs(match_sig)));
    figure;imagesc(na_lin(kk),r_lin,abs(match_sig1))
    xlabel('时间/s'); ylabel('距离/m'); title('回波脉冲压缩结果')
    set(gca,'FontSize',14);

    RD_sig = fftshift(fft(match_sig,[],2),2);
    RD_sig = (max(max(abs(RD_sig))) - abs(RD_sig))/max(max(abs(RD_sig)));
    figure;imagesc(fd_lin,r_lin,abs(RD_sig));
    xlabel('多普勒/Hz'); ylabel('距离/m'); title('回波距离-多普勒结果')
    set(gca,'FontSize',14);
%{
    %% 解模糊
    [D1,inx1,iny1] = my_Cfar(0,match_sig(:,1),50,30); 
    record = zeros(1,tar_num);
    dis = zeros(1,tar_num);
    t_rec = zeros(1,tar_num);
    for ii = 1:tar_num
        [x,y] = find(abs(match_sig(iny1,1)) == max(abs(match_sig(iny1,1))));
        if(iny1(x(1))>80 && iny1(x(1))<(length(match_sig(:,1)) - 80))
            match_sig(iny1(x(1))-80:iny1(x(1))+80,1) = 0;
        elseif(iny1(x(1))<80)
            match_sig(1:iny1(x(1))+80,1) = 0;
        else
            match_sig(iny1(x(1))-80:length(match_sig(:,1)),1) = 0;
        end
        record(ii) = iny1(x(1));
        t_rec(ii) = t(record(ii));
        dis(ii) = r_lin(record(ii));
    end
    total_t(kk,:) = t_rec;
    total_dis(kk,:) = dis;
end
for ii = 1:tar_num
    real_d(ii) = sol_amb(R_maxUnAmbi,total_dis(:,ii)); 
end

%}
end
