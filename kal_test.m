clear;
close all  
getd = @(p)path(path,p);
getd('fire_control_subpro/');
Tr = 1/5000;
F_kal = [1,0,0,Tr,0,0; 0,1,0,0,Tr,0; 0,0,1,0,0,Tr; 0,0,0,1,0,0; 0,0,0,0,1,0;0,0,0,0,0,1];   %状态转移矩阵
Q_kal = eye(6,6)*10;                                                                   %噪声干扰协方差矩阵
P = eye(6,6);                                                                               %状态协方差矩阵                    
R_kal = 0.5;                                                                                %观测噪声方差
u_kal = [0;0;0;0;0;0];                                                                      %控制向量
B_kal = [(Tr^2)/2;(Tr^2)/2;(Tr^2)/2;Tr;Tr;Tr];                                              %加速度向量
%H_kal = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0;0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1]   ; 
H_kal = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0];
load obe_val
load x_tra
load real_x
[m,N] = size(dis_rec1);
obe_v = zeros(m,N);
x = zeros(6,N+1);

for ii = 1:N-1
  %  obe_v(:,ii+1) = (dis_rec1(:,ii+1) - dis_rec1(:,ii))/Tr;
end
obe_val = [dis_rec1];
for ii = 1:N
    [x(:,ii+1),P] = my_kalman_filter(x(:,ii),P,H_kal,F_kal,R_kal,Q_kal,u_kal,B_kal,obe_val(:,ii));
end
result = soft_fil(dis_rec1(1,:),32);
error1 = sum(abs(result(1,64:5*64) - real_dis(1,64:5*64)))/5/64;
figure;
plot(Tr:Tr:6*64*Tr,real_dis(1,1:6*64))
hold on
plot(Tr:Tr:6*64*Tr,x(1,1:6*64))
plot(Tr:Tr:6*64*Tr,result(1,1:6*64))
legend('真实值','卡尔曼','观测值')