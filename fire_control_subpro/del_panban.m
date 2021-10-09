%该函数作用是删去一个峰的所有值（仅限于左右递减的峰）
function [result] = del_panban(data,x)
right_down_flag = 0;
left_down_flag = 0;
N = length(data);
now_left = data(x);
%峰左侧
for ii = 1:x-1
    if(now_left >= data(x-ii))
        now_left = data(x-ii);
        continue
    else
       left_down_flag = 1;
       index_left = ii-1;
       break
    end
end
%右侧
now_right = data(x);
for kk = 1:N-x
    if(now_right >= data(x+kk))
        now_right = data(x+kk);
        continue
    else
       right_down_flag = 1;
       index_right = kk-1;
       break
    end
end
if(left_down_flag == 0)
    index_left = x-1;
end
if(right_down_flag == 0)
    index_right = N-x;
end
data(x-index_left:x+index_right) = 0;
result = data;

