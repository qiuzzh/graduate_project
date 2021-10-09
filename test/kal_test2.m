close all
clear
t = linspace(0,10,101);
delta = t(2) - t(1);
N = length(t);
noise=20*randn(1,N); %方差为1的高斯噪声
noise2=20*randn(1,N); %方差为1的高斯噪声
g1 = 5; g2 = 0;
v1 = ones(1,N)*20 + g1*t;
v2 = ones(1,N)*10 + g2*t;
Z1 = v1.*t + (g1*t.^2)/2;
Z2 = v2.*t + (g2*t.^2)/2;
Z=[Z1;Z2;v1;v2];    %观测值（只有位置信息）
Z=Z+noise;   
X=[0;0;0;0];            %状态
P=eye(4,4)*1;        %状态协方差矩阵
F=[1 0 delta 0;0 1 0 delta;0 0 1 0;0 0 0 1];        %状态转移矩阵
Q=eye(4,4)*0.01;  %状态转移协方差矩阵
H=[1 0 0 0 ; 0 1 0 0;0 0 1 0;0 0 0 1];                %观测矩阵(H*X后只剩下位置信息，没有速度)
R=1;                    %观测噪声方差
a_M = [(g1.*delta.^2)/2;(g2.*delta.^2)/2;g1.*delta;g2.*delta]; 

x = zeros(4,N);
for i=1:N
    X_=F*x(:,i) + a_M;
    P_=F*P*F'+Q;
    K=P_*H'/(H*P_*H'+R);
    x(:,i+1)=X_+K*(Z(:,i)-H*X_);
    P=(eye(4)-K*H)*P_;
    
    %plot(X(1),X(2),'x');    %画点，横轴表示位置，纵轴表示速度
end

figure;plot(t,x(1,1:N),'k:.');
hold on 
plot(t,Z(1,:),'k--')
plot(t,Z1,'k')
xlabel("t\s")
ylabel("x轴位移/m")
legend("滤波值","观测值","真实值")
figure;plot(1:N+1,x(3,:),'k:.');

hold on 
plot(1:N,Z(3,:),'k--')
plot(1:N,v1,'k')
xlabel("t\s")
ylabel("x方向速度/m/s")
legend("滤波值","观测值","真实值")

error1 = sum(abs(x(1,1:N) - Z1))/N;
error2 = sum(abs(x(3,1:N) - v1))/N;