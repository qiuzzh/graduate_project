function [result] = soft_fil(data,sum_N)
% N Æ½»¬×ÜÊý

[M,N] = size(data);
result = zeros(M,N);
for ii = 1:N
    if(ii <= sum_N/2)
        result(1,ii) = data(1,ii);
    elseif(ii >= N-sum_N/2)
        result(1,ii) = data(1,ii);
    else
        total1 = sum(data(1,ii - sum_N/2:ii - 1));
        total2 = sum(data(1,ii + 1:ii + sum_N/2));
        result(1,ii) = (total1+total2)/(sum_N);
    end    
end

