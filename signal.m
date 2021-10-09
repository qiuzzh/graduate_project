%%%%%%% 雷达信号时域频域仿真

%%% 信号的DFT，注意时域的采样点与fft的点数应一致
clear;close all

%% LFM 、 单载频
T = 5e-6;           %脉宽
f0 = 1e7;           %载频
fs = 4*f0;          %采样率
N = fs*T;           %采样点数
N1 = fs*T*2;        %采样点数
t = (0:N-1)/fs;     %时间
t1 = (0:N1-1)/fs;   %时间2
y = square(pi*t1/T,50);
y = y + (y==-1);
y1 = y.*(exp(1i*2*pi*f0*t1));     %矩形*载频（注意复信号只有单边频谱，所以这里乘两个复信号）
%y1 = cos(2*pi*f0*t);
B = 1e7;         %带宽
K = B/T;        %调频斜率
f = linspace(0,1,N)*fs;
st = exp(1i*pi*2*(f0*t + K*(t).^2/2))/sqrt(T);          %复信号线性调频，所以只有单边频谱
%st = cos(2*pi*f0*t);
F1 = fft(st,N);
figure;plot(t,real(st));title('线性调频');xlabel('t')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;plot(f,abs(  F1));title('线性调频频谱');xlabel('f/Hz')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

xlabel('f/Hz');
F2 = fftshift(fft(y1,N1));
f1 = linspace(-0.5,0.5,N1)*fs;
amp_y1 = abs(F2/N1).^2;
figure;plot(f1,abs(amp_y1));title('单载频频谱');xlabel('f/Hz');ylabel('幅度')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;plot(t1,y1);title('单载频');xlabel('f/Hz');ylabel('幅度')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

%% 相位编码
T2 = 1e-7;                  %脉宽
len_phase = 13;             %13巴克码
bake13 = [1,1,1,1,1,-1,-1,1,1,-1,1,-1,1];
N3 = fs*T2*len_phase;       %时间总长度
t = (0:N3-1)/fs;            %时间
fi = zeros(1,N3);
index_bake = 0;
for i = 1:N3
    if(mod(t(i),T2) == 0)
        index_bake = index_bake + 1;
    end
    fi(i) = (bake13(index_bake) == -1)*pi;   %每个时刻的相位
end
st = (1/sqrt(T*len_phase)).*exp(1i*fi).*(exp(1i*2*pi*f0*t));
Fst = fftshift(fft(st,N3*2))/N3;
f3 = linspace(-0.5,0.5,N3*2)*fs;
figure;plot(t,st);title('相位编码时域');xlabel('t/s');ylabel('幅度')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;plot(f3,abs(Fst));title('相位编码频谱');xlabel('f/Hz');ylabel('幅度')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

%% 单频矩形脉冲相参串
Fc = 50;
Fs = 20*Fc;
Te = 0.1;
Tr = 0.5;
t = linspace(0,Te,Te*Fs);
tr = linspace(0,10*Tr,10*Tr*Fs);
mai1 = exp(1i*2*pi*Fc*tr);
mai2 = zeros(1,round(Tr*Fs));
mai2(1,1:length(t)) = 1;
for i = 0:9
    mai3(1,1+i*length(mai2):i*length(mai2)+length(mai2)) = mai2; 
end
mai4 = mai3.*mai1;
%figure;plot(linspace(0,Tr,Tr*Fs),mai2);
%figure;plot(linspace(0,10*Tr,10*Tr*Fs),mai3)
figure;plot(tr,mai4);xlabel('t/s');ylabel('幅度')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;plot(linspace(-Fs/2,Fs/2,length(mai4)),abs(fftshift(fft(mai4))));xlabel('f\Hz');ylabel('幅度')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

%% 模糊函数

%%%%%%%%%%%%%%%脉冲串
tao = linspace(-0.1*Tr,0.1*Tr,0.1*Tr*Fs);
tao_N = length(tao);
fd = linspace(-Fc/4,Fc/4,length(tao));
Xtd = zeros(tao_N,tao_N);
for ii = 1:tao_N
    for jj = 1:tao_N
        mai_t = exp(1i*2*pi*Fc*(tr + tao(ii)));
        mai_tt = mai_t.*mai3;
        Xtd(ii,jj) = abs(real(sum( conj(mai_tt).*mai4.*exp(-1i*2*pi*fd(jj)*tr) )));    
    end
end
figure;plot(tr,mai_tt);
figure;mesh(tao,fd,Xtd);title('脉冲串模糊函数图');xlabel('fd\Hz');ylabel('τ\s')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;contour(tao,fd,sqrt(Xtd));title('脉冲串模糊函数等值线图')
%%%%%%%%%%%%%%%%%% 连续波
t = (0:N-1)/fs;     %时间
tao = (-N:N)/fs;
tao_N = size(tao,2);
fd = linspace(-B,B,tao_N);
fd_N =size(fd,2);
Xt = zeros(1,tao_N);Xd = zeros(1,fd_N);Xtd = zeros(tao_N,fd_N);
for i = 1:tao_N         %距离模糊函数
    xt = conj(exp(1i*pi*2*(f0*(t-tao(i)) + K*((t-tao(i))).^2/2))/sqrt(T)).*exp(1i*pi*2*(f0*t + K*(t).^2/2))/sqrt(T);
    Xt(i) = abs(sum(xt)*(t(2) - t(1)))^2;
end
for i = 1:fd_N          %频率模糊函数
    xd = conj(exp(1i*pi*2*(f0*(t) + K*(t).^2/2))/sqrt(T)) .* exp(1i*pi*2*(f0*t + K*(t).^2/2))/sqrt(T).*exp(1i*2*pi*fd(i)*t);
    Xd(i) = abs(sum(xd)*(t(2) - t(1)))^2;
end
syms x
for i = 1:tao_N
    for j = 1:fd_N
        Xtd(i,j) = abs(exp(1j*tao(i)*fd(j))*(1-abs(tao(i)/T))*sin(pi*T*(K*tao(i) + fd(j))*(1-abs(tao(i))/T))/(pi*T*(K*tao(i)+fd(j))*(1-abs(tao(i)/T))))^2;
    end
end
X = Xt'*Xd;
figure;plot(tao,Xt);title('距离模糊函数');figure;plot(fd,Xd);title('频率模糊函数')
figure;mesh(tao,fd,Xtd);title('LFM模糊函数图');xlabel('fd\Hz');ylabel('τ\s')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;contour(tao,fd,sqrt(Xtd));title('LFM模糊函数等值线图')

%%%%%%%%%%%%%%%%%% 单载频方波
tao = (-N:N)/fs;
tao_N = size(tao,2);
fd = linspace(-f0,f0,tao_N);
Xt = zeros(tao_N,tao_N);
for i = 1:tao_N
    for j = 1:tao_N
        Xtd(i,j) = abs((1-abs(tao(i))/T)*(sin(pi*fd(j)*(T-abs(tao(i)))))/(pi*fd(j)*(T - abs(tao(i)))))^2;
    end
end
figure;mesh(tao,fd,Xtd);title('单载频模糊函数图');xlabel('fd\Hz');ylabel('τ\s')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;contour(tao,fd,sqrt(Xtd));title('单载频模糊函数等值线图')


%%%%%%%%%%%%%%%%%%% 相位
t = (0:N3-1)/fs;            %时间
fd = 10;
[ambig] = ambiguity_code(bake13,10);
freq = linspace(-6,6,size(ambig,1));
N = size(bake13,2);
tao = N;
samp = size(bake13,2)*10;
n = ceil(log(samp)/log(2));
nfft = 2^n;
delay = linspace(-N-2,N,nfft);
figure;mesh(delay,freq,ambig);title('相位调频模糊函数图');xlabel('fd\Hz');ylabel('τ\s')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;contour(delay,freq,sqrt(ambig));title('相位调频模糊函数等值线图')
