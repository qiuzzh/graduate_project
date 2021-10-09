function [s1,R_lin,a_lin] = MatchFilter(s,Tr,Fs,Tp,Fc,K,B)
%   MatchFilter_LFM：线性调频回波信号匹配滤波函数
%   Input:  s：  回波信号Cube
%           Tr： 脉冲重复时间(信号采样长度对应时间)
%           Fs： 采样率
%           Tp:  脉冲宽度
%   Output： s1：脉压后结果
%           R_lin:  距离标尺
%           a_lin：方位时间标尺
%% 匹配滤波方式1()

%%%% 注意采样率与带宽大小差距不能太大，否则匹配效果差
Delta_t=1/Fs;
t_set_Tp=(Delta_t:Delta_t:Tp)'; 
N=ceil(length(t_set_Tp)/2)*2;
[nrn,nan] = size(s);
sig = zeros(nrn,1);
sig(1:N) = exp(2*1i*pi*Fc*(t_set_Tp)).*exp(1i*pi*K*(t_set_Tp).^2);       %LFM时域
%figure;plot(Delta_t:Delta_t:Tr,sig);
sig_ft = fftshift(fft(sig,[],1),1);
S=fftshift(fft(s,[],1),1);
S = S.*conj(sig_ft);
Win = zeros(nrn,1);
win_l = round(nrn*B/Fs);
%{
if(mod(win_l,4)==1)
    win_l = win_l - 1;
elseif(mod(win_l,4)==2)
    win_l = win_l + 2;
elseif(mod(win_l,4)==3)
    win_l = win_l + 1;
end
%}
if(Fs<2*Fc)
    Win(nrn/2:nrn/2+win_l-1) = hamming(win_l);     %加窗注意窗长度要和采样率与带宽大小相对应，否则无用。
else
    if(nrn/2+Fc/(Fs/2)*nrn/2+win_l <= nrn)
        Win(nrn/2+Fc/(Fs/2)*nrn/2:nrn/2+Fc/(Fs/2)*nrn/2+win_l) = hamming(win_l);
    else
        win1 = hamming(win_l);
        Win((nrn/2+Fc/(Fs/2)*nrn/2):nrn) = win1(1:nrn-(nrn/2+(Fc/(Fs/2)*nrn/2))+1);
        last = win_l - length(win1(1:nrn-(nrn/2+Fc/(Fs/2)*nrn/2)+1));
        Win(1:last) = win1(nrn-(nrn/2+Fc/(Fs/2)*nrn/2)+2:win_l);
    end
end
%Win = ones(nrn,1);
%figure;plot(linspace(-Fs/2,Fs/2,nrn),abs(S));
%figure;plot(linspace(-Fs/2,Fs/2,nrn),abs(S.*Win));
s1=ifft(fftshift(S.*Win,1),[],1);
% figure;imagesc(abs(fft(s1,[],2)))
c = 3e8;
dR = c/2/Fs;
[nrn,nan] = size(s1);                       %%%nrn为采样点数，nan为128个脉冲的回波
R_lin = (1:nrn)*dR + 0 - c*Tp/2/2;          %%%每个采样点之间的距离间距，发射脉冲期间不能收
a_lin = 1:nan;

% figure;imagesc(a_lin,R_lin,abs(fftshift(s1,1)))
% figure;imagesc(a_lin,R_lin,abs((s)))
%% 匹配滤波方式2
% sig = zeros(nrn,1);
% sig(1:N) = exp(1i*pi*K*(t_set_Tp-Tp/2).^2);
% sig_ft = fftshift(fft(sig,[],1),1);
% S=fftshift(fft(s,[],1),1);
% S = S.*conj(sig_ft);
% Win = hamming(nrn);
% s1=ifft(ifftshift(S.*Win,1),[],1);
% dR = c/2/Fs;
% [nrn,nan] = size(s1);
% R_lin = (1:nrn)*dR + R_start;
% a_lin = 1:nan;
% figure;imagesc(a_lin,R_lin,abs(s1))
% figure;imagesc(a_lin,R_lin,abs((s)))