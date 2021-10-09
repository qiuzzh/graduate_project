%% 用于线性预测轨迹
function [result] = my_predict(data)
[M,N] = size(data);
ave_cha = zeros(M,1);
for ii = 1:M
    sum = 0;
    for jj = 2:N
        cha = data(ii,jj) - data(ii,jj-1);
        sum = cha + sum;
    end
    ave_cha(ii,1) = sum/(N-1);
end
result = data(:,end) + ave_cha;