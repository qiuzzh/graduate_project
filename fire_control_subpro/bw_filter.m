function [result] = bw_filter(sig,B,fc,fs)
[m,n] = size(sig);
F_sig = fftshift(fft(sig));
%figure;plot(linspace(-fs/2,fs/2,m),real(sig));
%figure;plot(linspace(-fs/2,fs/2,m),abs(F_sig));
win = zeros(m,n);
win(round(m/2+fc*m/fs)-round(B*5/4*m/fs)/2:round(m/2+fc*m/fs)+round(B*5/4*m/fs)/2,1) = 1;
%figure;plot(linspace(-fs/2,fs/2,m),win);
result = F_sig.*win;
%figure;plot(linspace(-fs/2,fs/2,m),abs(result));
result = ifft(fftshift(result));
%figure;plot(linspace(-fs/2,fs/2,m),result);
