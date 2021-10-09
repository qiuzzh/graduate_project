function [ambig] = ambiguity_code(input,fd)
N = size(input,2);
tao = N;                            %延时个数
code = input;                       %输入
samp = size(code,2)*10;             %取样信号个数
n = ceil(log(samp)/log(2));         
nfft = 2^n;                         %fft点数
u(1:nfft) = 0;                      
j = 0;
for index = 1:10:samp
    j = j+1;
    u(index:index+10-1) = code(j);  %采样信号
end
v = u;
delay = linspace(0,5*tao,nfft);     %延时
freq_del = 2*fd/tao/100;              %多普勒频率频移
j = 0;
vfft = fft(v,nfft);                 %采样信号fft
for freq = -fd/tao:freq_del:fd/tao
    j = j+1;
    exf = exp(1i*2*pi*freq.*delay); 
    u_times = u.*exf;               %频移、时延后信号
    ufft = fft(u_times,nfft);       %频移、时延后信号fft
    prod = ufft.*conj(vfft);        %频域相乘
    ambig(j,:) = fftshift(abs(ifft(prod)));%时域卷积（积分）
end