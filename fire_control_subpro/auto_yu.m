function [yu] = auto_yu(data)
[M,N] = size(data);
p = data/M/N;
mg = sum((0:M).*p);
delta_now = 0;
for yu1 = 1:M                     %������ֵ
        m1 = 0;m2 = 0;
        P1 = sum(p(1:yu1))/M/N;     %ע�ⷶΧ����1��1�����кܴ�仯
        P2 = sum(p(yu1+1:end))/M/N;
        for ii = 1:M
            if(ii<=yu1)
                m1 = (ii-1)*p(ii) + m1;
            elseif(ii>yu1)
                m2 = (ii-1)*p(ii) + m2;
            end
        end
        m1 = m1/P1;m2 = m2/P2;m3 = m3/P3;   
        delta = P1*(m1 - mg)^2 + P2*(m2 - mg)^2;   %����
        if(delta > delta_now)           %ѡ��������
            delta_now = delta;
            k1 = yu1;
        end
end
yu = k1;