%%%%%%% �״��ź�ʱ��Ƶ�����

%%% �źŵ�DFT��ע��ʱ��Ĳ�������fft�ĵ���Ӧһ��
clear;close all

%% LFM �� ����Ƶ
T = 5e-6;           %����
f0 = 1e7;           %��Ƶ
fs = 4*f0;          %������
N = fs*T;           %��������
N1 = fs*T*2;        %��������
t = (0:N-1)/fs;     %ʱ��
t1 = (0:N1-1)/fs;   %ʱ��2
y = square(pi*t1/T,50);
y = y + (y==-1);
y1 = y.*(exp(1i*2*pi*f0*t1));     %����*��Ƶ��ע�⸴�ź�ֻ�е���Ƶ�ף�����������������źţ�
%y1 = cos(2*pi*f0*t);
B = 1e7;         %����
K = B/T;        %��Ƶб��
f = linspace(0,1,N)*fs;
st = exp(1i*pi*2*(f0*t + K*(t).^2/2))/sqrt(T);          %���ź����Ե�Ƶ������ֻ�е���Ƶ��
%st = cos(2*pi*f0*t);
F1 = fft(st,N);
figure;plot(t,real(st));title('���Ե�Ƶ');xlabel('t')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;plot(f,abs(  F1));title('���Ե�ƵƵ��');xlabel('f/Hz')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

xlabel('f/Hz');
F2 = fftshift(fft(y1,N1));
f1 = linspace(-0.5,0.5,N1)*fs;
amp_y1 = abs(F2/N1).^2;
figure;plot(f1,abs(amp_y1));title('����ƵƵ��');xlabel('f/Hz');ylabel('����')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;plot(t1,y1);title('����Ƶ');xlabel('f/Hz');ylabel('����')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

%% ��λ����
T2 = 1e-7;                  %����
len_phase = 13;             %13�Ϳ���
bake13 = [1,1,1,1,1,-1,-1,1,1,-1,1,-1,1];
N3 = fs*T2*len_phase;       %ʱ���ܳ���
t = (0:N3-1)/fs;            %ʱ��
fi = zeros(1,N3);
index_bake = 0;
for i = 1:N3
    if(mod(t(i),T2) == 0)
        index_bake = index_bake + 1;
    end
    fi(i) = (bake13(index_bake) == -1)*pi;   %ÿ��ʱ�̵���λ
end
st = (1/sqrt(T*len_phase)).*exp(1i*fi).*(exp(1i*2*pi*f0*t));
Fst = fftshift(fft(st,N3*2))/N3;
f3 = linspace(-0.5,0.5,N3*2)*fs;
figure;plot(t,st);title('��λ����ʱ��');xlabel('t/s');ylabel('����')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;plot(f3,abs(Fst));title('��λ����Ƶ��');xlabel('f/Hz');ylabel('����')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

%% ��Ƶ����������δ�
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
figure;plot(tr,mai4);xlabel('t/s');ylabel('����')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;plot(linspace(-Fs/2,Fs/2,length(mai4)),abs(fftshift(fft(mai4))));xlabel('f\Hz');ylabel('����')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

%% ģ������

%%%%%%%%%%%%%%%���崮
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
figure;mesh(tao,fd,Xtd);title('���崮ģ������ͼ');xlabel('fd\Hz');ylabel('��\s')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;contour(tao,fd,sqrt(Xtd));title('���崮ģ��������ֵ��ͼ')
%%%%%%%%%%%%%%%%%% ������
t = (0:N-1)/fs;     %ʱ��
tao = (-N:N)/fs;
tao_N = size(tao,2);
fd = linspace(-B,B,tao_N);
fd_N =size(fd,2);
Xt = zeros(1,tao_N);Xd = zeros(1,fd_N);Xtd = zeros(tao_N,fd_N);
for i = 1:tao_N         %����ģ������
    xt = conj(exp(1i*pi*2*(f0*(t-tao(i)) + K*((t-tao(i))).^2/2))/sqrt(T)).*exp(1i*pi*2*(f0*t + K*(t).^2/2))/sqrt(T);
    Xt(i) = abs(sum(xt)*(t(2) - t(1)))^2;
end
for i = 1:fd_N          %Ƶ��ģ������
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
figure;plot(tao,Xt);title('����ģ������');figure;plot(fd,Xd);title('Ƶ��ģ������')
figure;mesh(tao,fd,Xtd);title('LFMģ������ͼ');xlabel('fd\Hz');ylabel('��\s')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;contour(tao,fd,sqrt(Xtd));title('LFMģ��������ֵ��ͼ')

%%%%%%%%%%%%%%%%%% ����Ƶ����
tao = (-N:N)/fs;
tao_N = size(tao,2);
fd = linspace(-f0,f0,tao_N);
Xt = zeros(tao_N,tao_N);
for i = 1:tao_N
    for j = 1:tao_N
        Xtd(i,j) = abs((1-abs(tao(i))/T)*(sin(pi*fd(j)*(T-abs(tao(i)))))/(pi*fd(j)*(T - abs(tao(i)))))^2;
    end
end
figure;mesh(tao,fd,Xtd);title('����Ƶģ������ͼ');xlabel('fd\Hz');ylabel('��\s')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;contour(tao,fd,sqrt(Xtd));title('����Ƶģ��������ֵ��ͼ')


%%%%%%%%%%%%%%%%%%% ��λ
t = (0:N3-1)/fs;            %ʱ��
fd = 10;
[ambig] = ambiguity_code(bake13,10);
freq = linspace(-6,6,size(ambig,1));
N = size(bake13,2);
tao = N;
samp = size(bake13,2)*10;
n = ceil(log(samp)/log(2));
nfft = 2^n;
delay = linspace(-N-2,N,nfft);
figure;mesh(delay,freq,ambig);title('��λ��Ƶģ������ͼ');xlabel('fd\Hz');ylabel('��\s')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;contour(delay,freq,sqrt(ambig));title('��λ��Ƶģ��������ֵ��ͼ')
