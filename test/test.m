close all;clear;
Fc=1e6                     ;	% 中心频率【Hz】
B=6e5                      ;	% 带宽【Hz】
Fs=2*B                   ;	% 采样率【Hz】
Tr=1e-3                     ;	% 脉冲重复周期【秒】 2e-2
Tp=Tr                     ;	% 脉冲宽度【微秒】
K = B/Tp;
t = (1/Fs:1/Fs:Tp);
tr = (1/Fs:1/Fs:Tr);
f = linspace(-Fs/2,Fs/2,length(tr));
dely = 0.2*Tr;
s2 = exp(2*pi*1i*Fc*(t-dely)).*exp(pi*1i*K*(t-dely).^2);
s1 = exp(2*pi*1i*Fc*(t)).*exp(pi*1i*K*(t).^2);
sig1 = zeros(length(tr),1);
sig1(1:length(t),1) = s1;
sig2 = zeros(length(tr),1);
sig2(1:length(t),1) = s2;
s_f1 = fftshift(fft(sig1,[],1));
s_f2 = fftshift(fft(sig2,[],1));
figure;plot(f,abs(s_f2));
xlabel('f/Hz');ylabel('幅度')
figure;plot(t,sig2);
xlabel('t/s');ylabel('幅度')
S = s_f2.*conj(s_f1);
F = ifft(fftshift(S),[],1);
figure;plot(t,s1);xlabel('t/s');ylabel('幅度')
figure;plot(f,abs(fftshift(fft(sig1))));
[sm,p1,p2] = MatchFilter(sig2,Tr,Fs,Tp,Fc,K,B);
figure;plot(tr,sig1);xlabel('t/s');ylabel('幅度')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

figure;plot(tr,abs(sm));xlabel('t/s');ylabel('幅度')
set(get(gca,'XLabel'),'FontSize',16);set(get(gca,'YLabel'),'FontSize',16);set(get(gca,'TITLE'),'FontSize',15);set(gca,'fontsize',14);

%{
load ang
load dis_rec1
load real_dis
load x_record
load total_data
x_soft = squeeze(total_data(:,1,:));
dis_door = 2;

%% 跟踪测试
[M,N] = size(dis_rec1);
tra_pro = zeros(M,N);
start_point = total_data(:,1,1);
tra_pro(:,1) = start_point;
if_rock = zeros(1,N);
lose_num = 0;
pre_num = 10;
for ii = 2:N
    for kk = 1:3
        now = total_data(:,kk,ii);
        sigma = norm(now - tra_pro(:,ii-1));
        if(sigma < dis_door)
            tra_pro(:,ii) = total_data(:,kk,ii);
            if_rock(ii) = 1;
            lose_num = 0;
            break
        end
    end
    if(if_rock(ii)==0)
        tra_pro(:,ii) = my_predict(tra_pro(:,ii-pre_num:ii-1));
        lose_num = lose_num + 1;
    end
    if(lose_num > 20)
        disp('lose')
        break
    end
end

figure;hold on
for ii = 1:N
    plot3(tra_pro(1,ii),tra_pro(2,ii),tra_pro(3,ii),'.')
    %drawnow
end
hold off
figure;hold on
for ii = 1:N
    plot3(total_data(1,:,ii),total_data(2,:,ii),total_data(3,:,ii),'.')
    %drawnow
end
%}