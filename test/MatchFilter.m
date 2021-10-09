function [s1,R_lin,a_lin] = MatchFilter(s,Tr,Fs,Tp,Fc,K,B)
%   MatchFilter_LFM�����Ե�Ƶ�ز��ź�ƥ���˲�����
%   Input:  s��  �ز��ź�Cube
%           Tr�� �����ظ�ʱ��(�źŲ������ȶ�Ӧʱ��)
%           Fs�� ������
%           Tp:  ������
%   Output�� s1����ѹ����
%           R_lin:  ������
%           a_lin����λʱ����
%% ƥ���˲���ʽ1()

%%%% ע�������������С��಻��̫�󣬷���ƥ��Ч����
Delta_t=1/Fs;
t_set_Tp=(Delta_t:Delta_t:Tp)'; 
N=ceil(length(t_set_Tp)/2)*2;
[nrn,nan] = size(s);
sig = zeros(nrn,1);
sig(1:N) = exp(2*1i*pi*Fc*(t_set_Tp)).*exp(1i*pi*K*(t_set_Tp).^2);       %LFMʱ��
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
    Win(nrn/2:nrn/2+win_l-1) = hamming(win_l);     %�Ӵ�ע�ⴰ����Ҫ�Ͳ�����������С���Ӧ���������á�
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
[nrn,nan] = size(s1);                       %%%nrnΪ����������nanΪ128������Ļز�
R_lin = (1:nrn)*dR + 0 - c*Tp/2/2;          %%%ÿ��������֮��ľ����࣬���������ڼ䲻����
a_lin = 1:nan;

% figure;imagesc(a_lin,R_lin,abs(fftshift(s1,1)))
% figure;imagesc(a_lin,R_lin,abs((s)))
%% ƥ���˲���ʽ2
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