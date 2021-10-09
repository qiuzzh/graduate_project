%%%%%%
%”√¿¥≤‚ ‘µƒŒƒº˛
%%%%%%
close all
clear
getd = @(p)path(path,p);
getd('fire_control_subpro/');

Tr = 1e-3;
Fc = 10e6;
Te = 1e-4;
B=5e6;
Fs = 4*Fc;
c = 3e8;
K = B/Te;
Pfa = 1e-6;
lambda = c/Fc;
prf = 1/Tr;
t = linspace(0,Te,ceil(Te*Fs));
tr = linspace(0,Tr,ceil(Tr*Fs));
cpi_sig = zeros(ceil(Tr*Fs),64);
tr = linspace(0,Tr,ceil(Tr*Fs));
F = linspace(-Fs/2,Fs/2,ceil(Tr*Fs));
v = 3000;

for ii = 1:128
    ta = (ii-1)*Tr + t;
    signal = zeros(ceil(Tr*Fs),1);
    dely = 2*(5000 + v*ta)/c;
    sig = 4*exp(2*pi*Fc*(ta-dely)*1i).*exp(1i*pi*K*(t-dely ).^2);
    noise = randn(size(signal)) + 1i*randn(size(signal) ) ; 
    signal(1:length(sig),1) = sig;
    s_s = signal + noise;
    [s_match,R_lin,a_lin] = MatchFilter(s_s,Tr,Fs,Te,Fc,K); %%%∆•≈‰¬À≤®
   figure;plot(linspace(-Fs/2,Fs/2,length(s_match)),abs(fftshift(fft(signal))))
    %figure;plot(tr,s_match)
    [D1,inx1,iny1] = my_Cfar(Pfa,s_match,ceil(length(s_match)/200),ceil(length(s_match)/500)); 
    cpi_sig(:,ii) = s_match;
end
figure;plot(F,abs(fftshift(fft(s_s))));
figure;plot(tr,s_s);
figure;plot(tr,abs(cpi_sig(:,2))) 
figure;plot(F,abs(fftshift(fft(cpi_sig(:,2)))))
[max_x,max_y] = find(abs(s_match) == max(abs(s_match)));

start = dely/Tr*ceil(Tr*Fs);
ff_sig = fftshift(fft(cpi_sig(max_x,:)));
ff = linspace(-prf/2,prf/2,128);    
figure;plot(ff,abs(ff_sig))
[last1,last2] = find(abs(ff_sig) == max(abs(ff_sig)));
ff(last2)
vd = ff(last2)*lambda/2
%real_fd = -2*v/lambda


%[s_match2,R_lin,a_lin] = MatchFilter(noise,Tr,Fs,Te,Fc,K); %%%∆•≈‰¬À≤®
%figure;plot(tr,s_match)
%figure;plot(tr,s_match2)
