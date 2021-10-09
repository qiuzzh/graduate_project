%% 由于多普勒频率相对于中心频率很小很小，且他们之间相对的比例是固定的，
%多普勒测速测试文件
%%%%%
% 所以改变中心频率无效，要改变的是信号脉宽，要足够宽即可
clear
close all
Tr = 1e-1;
Fc = 10e6;
Te = 1e-2;
B=1e6;
Fs = 2*Fc;
c = 3e8;
K = B/Te;
Pfa = 1e-6;
lambda = c/Fc;
dely = 0/c;
t = linspace(0,Te,Te*Fs);
sen_sig = 2*exp(2*pi*Fc*(t-Te/2-dely)*1i).*exp(1i*pi*K*(t-Te/2-dely).^2);
sig = 2*exp(2*pi*Fc*(t-dely - 2*3000*t/c)*1i).*exp(1i*pi*K*(t-dely - 2*3000*t/c).^2);
%{
cha_sig = sen_sig./sig;
N = length(t)*40;
cha_f = abs(fftshift(fft(cha_sig,N)));

test = exp(2*pi*1i*t*10);
test_f = abs(fftshift(fft(test,N)));
F = linspace(-Fs/2,Fs/2,N);
%plot(F,test_f)
[inx,iny] = max(cha_f);
%}


