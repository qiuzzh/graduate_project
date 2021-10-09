%% 中国余数定理
function [b] = find_b(m,M)
N = length(m);
now = 1;
for ii = 1:N
    now = m(ii)*now;
end
b1 = 1;
while(1)
  if(mod(b1*now,M) == 1)
      b = b1;
      break
  else
      b1 = b1 + 1;
  end
end
