function [XT,result,inx,iny] = my_Cfar(Pfa,signal,N,pro)

%N = 200; %总单元数
%pro = 100;   %保护单元数
k = 5;
if(mod(N,2)==1)
    N = N + 1;
end
if(mod(pro,2)==1)
    pro = pro+1;
end
%alpha = N.^(Pfa.^(-1./N)-1);
%index = 1+N/2+pro/2:length(signal)-N/2-pro/2;
index = 1:length(signal);
XT = zeros(1,length(signal));
result = zeros(1,length(signal));
for ii = index
    if(ii < 1+N/2 +pro/2 )
        cell = abs(signal(ii+1+pro/2:ii+N/2+pro/2,1));
        z = sum(cell)/length(cell);
    elseif(ii>length(signal) - N/2 - pro/2)
        cell = abs(signal(ii--pro/2-N/2:ii-1-pro/2,1));
        z = sum(cell)/length(cell);
    else
        cell_left = abs(signal(ii-N/2-pro/2:ii-pro/2-1,1));
        cell_right = abs(signal(ii+pro/2+1:ii+N/2+pro/2,1));
        %cell_all = cat(2,cell_left,cell_right);
        %cell_sort = sort(cell_all);
        %cell_cat = sort(cat(2,cell_right,cell_left));
        %z = cell_sort(k,1);
        z = sum(cell_left)+sum(cell_right);
        z = z/N;            %ca-cfar
        %z = min([mean(cell_left),mean(cell_right)]);       %so_cfar
        %z = cell_cat(N/2);
    end
    XT(1,ii) = z*k;
    if(abs(signal(ii))>z*k)
        result(1,ii) = 1;
    else
        result(1,ii) = 0;
    end
end
%figure;plot(1:length(signal),abs(signal),'k')
%hold on
%plot(1:length(signal),abs(XT),'k--');xlabel('t\s');ylabel('幅度')
[inx,iny] = find(result == 1);
end

    