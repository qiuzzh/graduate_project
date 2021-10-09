function G = GetBeamGain(beam,a_ind,p_ind,tar_a, tar_p, pit_ang, azi_ang, ane_ang)
%   GetBeamGain: 通过天线方向图以及目标角度，得到该方向增益（主要进行插值）
%   Input：  beam：二维天线方向图，由BeanPattern产生
%           a_ind：天线方向图方位向索引
%           p_ind：天线方向图俯仰向索引
%           tar_a：目标方位向角度
%           tar_p：目标俯仰向角度
%           ane_ang; 天线波束范围
%   Output：G：目标方向的天线增益
%%
% 利用双线性插值，计算目标方向增益
if(tar_a >= azi_ang - ane_ang/2 && tar_a <= azi_ang + ane_ang/2 && tar_p >= pit_ang - ane_ang/2 && tar_p <= pit_ang + ane_ang/2)
    azi = sign(a_ind + azi_ang - tar_a);
    a_i = find(azi == -1 | azi == 0);
    a_ind_1 = a_i(end);
    if(a_ind_1 == size(beam,1))
        a_ind_1 = a_ind_1 - 1;
    end
    a_ind_2 = a_ind_1+1;

    pit = sign(p_ind + pit_ang - tar_p);
    p_i = find(pit == -1 | azi == 0);
    p_ind_1 = p_i(end);
    if(p_ind_1 == size(beam,1))
        p_ind_1 = p_ind_1 - 1;
    end
    p_ind_2 = p_ind_1+1;

    % 取值
    g1 = beam(a_ind_1,p_ind_1);
    g2 = beam(a_ind_1,p_ind_2);
    g3 = beam(a_ind_2,p_ind_1);
    g4 = beam(a_ind_2,p_ind_2);

    g_a_p1 = g1 + (g2-g1)*(tar_p-p_ind(p_ind_1))/(p_ind(p_ind_2)-p_ind(p_ind_1)); % 先进行pitch方向插值
    g_a_p2 = g3 + (g4-g3)*(tar_p-p_ind(p_ind_1))/(p_ind(p_ind_2)-p_ind(p_ind_1));

    G = g_a_p1 + (g_a_p2 - g_a_p1)*(tar_a-a_ind(a_ind_1))/(a_ind(a_ind_2)-a_ind(a_ind_1)); % 在进行azimuth方向插值
else
    G = 0;
end
end