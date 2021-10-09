function [yu] = auto_yu(data)
[M,N] = size(data);
p = data/M/N;
mg = sum((0:M).*p);
delta_now = 0;
for yu1 = 1:M                     %两个阈值
        m1 = 0;m2 = 0;
        P1 = sum(p(1:yu1))/M/N;     %注意范围，少1多1都会有很大变化
        P2 = sum(p(yu1+1:end))/M/N;
        for ii = 1:M
            if(ii<=yu1)
                m1 = (ii-1)*p(ii) + m1;
            elseif(ii>yu1)
                m2 = (ii-1)*p(ii) + m2;
            end
        end
        m1 = m1/P1;m2 = m2/P2;m3 = m3/P3;   
        delta = P1*(m1 - mg)^2 + P2*(m2 - mg)^2;   %方差
        if(delta > delta_now)           %选方差最大的
            delta_now = delta;
            k1 = yu1;
        end
end
yu = k1;