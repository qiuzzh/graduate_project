function [result,P_l] = my_kalman_filter(x,P,H,F,R,Q,u,B,obe_val)
% ori_val 状态初始值
% P 状态协方差矩阵
% H 观测矩阵
% Q 噪声干扰协方差矩阵
% R 观测噪声方差
% F 状态转移矩阵
% u 控制向量
% B 控制状态矩阵
% x 初始状态
x_ = F*x + B.*u;
P_ = F*P*F'+Q;
K = P_*H'/(H*P_*H'+ R);
x_l = x_ + K*(obe_val - H*x_);
I = eye(6,6);
P_l = (I-K*H)*P_;

result = x_l;
