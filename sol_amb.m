function [result] = sol_amb(flock,yu)
%flock 周期比（互质）
%yu 余数
N = length(flock);
now = 1;
for ii = 1:N
    now = flock(ii)*now;
end
M = zeros(1,N);
t = zeros(1,N);
for ii = 1:N
    M(1,ii) = now/flock(ii);
end
b = zeros(1,N);
c = zeros(1,N);
for ii = 1:N
    b(1,ii) = find_b(M(1,ii),flock(ii));
    c(1,ii) = b(1,ii)*M(1,ii);
end
temp = 0;
for ii = 1:N
    temp = c(1,ii)*yu(ii) + temp;
end
result = mod(temp,now);
%{
for ii =1:N
    t(1,ii) = mod(M(1,ii),flock(ii));
end
re = 0;
for ii = 1:N
    re = re + yu(ii)*t(1,ii)*M(1,ii);
end
result = mod(re,now);
%}
