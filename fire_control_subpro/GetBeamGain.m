function G = GetBeamGain(beam,a_ind,p_ind,tar_a, tar_p, pit_ang, azi_ang, ane_ang)
%   GetBeamGain: ͨ�����߷���ͼ�Լ�Ŀ��Ƕȣ��õ��÷������棨��Ҫ���в�ֵ��
%   Input��  beam����ά���߷���ͼ����BeanPattern����
%           a_ind�����߷���ͼ��λ������
%           p_ind�����߷���ͼ����������
%           tar_a��Ŀ�귽λ��Ƕ�
%           tar_p��Ŀ�긩����Ƕ�
%           ane_ang; ���߲�����Χ
%   Output��G��Ŀ�귽�����������
%%
% ����˫���Բ�ֵ������Ŀ�귽������
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

    % ȡֵ
    g1 = beam(a_ind_1,p_ind_1);
    g2 = beam(a_ind_1,p_ind_2);
    g3 = beam(a_ind_2,p_ind_1);
    g4 = beam(a_ind_2,p_ind_2);

    g_a_p1 = g1 + (g2-g1)*(tar_p-p_ind(p_ind_1))/(p_ind(p_ind_2)-p_ind(p_ind_1)); % �Ƚ���pitch�����ֵ
    g_a_p2 = g3 + (g4-g3)*(tar_p-p_ind(p_ind_1))/(p_ind(p_ind_2)-p_ind(p_ind_1));

    G = g_a_p1 + (g_a_p2 - g_a_p1)*(tar_a-a_ind(a_ind_1))/(a_ind(a_ind_2)-a_ind(a_ind_1)); % �ڽ���azimuth�����ֵ
else
    G = 0;
end
end