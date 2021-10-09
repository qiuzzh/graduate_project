function [result,P_l] = my_kalman_filter(x,P,H,F,R,Q,u,B,obe_val)
% ori_val ״̬��ʼֵ
% P ״̬Э�������
% H �۲����
% Q ��������Э�������
% R �۲���������
% F ״̬ת�ƾ���
% u ��������
% B ����״̬����
% x ��ʼ״̬
x_ = F*x + B.*u;
P_ = F*P*F'+Q;
K = P_*H'/(H*P_*H'+ R);
x_l = x_ + K*(obe_val - H*x_);
I = eye(6,6);
P_l = (I-K*H)*P_;

result = x_l;
