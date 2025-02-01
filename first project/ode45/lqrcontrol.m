clear all;
%% 系统参数
M = 0.2275;
m = 0.0923;
g = 9.8;
L = 0.185;
J = 0.00412;
%% 动态方程
% x=x1;dot(x)=x2;phi=x3;dot(phi)=x4;
% (M+m)*dot(x2)-m*l*dot(x4)=F;
% l*dot(x4)-dot(x2)+(J/(m*l))*dot(x4)-g*x3=0
a=J/(m*L);
fen_mu=M*L+M*a+m*a;

%% 状态空间矩阵
A=[
    0 1 0 0;
    0 0 m*g*L/fen_mu 0;
    0 0 0 1;
    0 0 (m+M)*g/fen_mu 0
    ];

B=[0;(L+a)/fen_mu;0;1/fen_mu];
C=eye(4);
D=0;
%% 建立系统
cart=ss(A,B,C,D);
%% LQR
Q = diag([10 1 10 1]);% x,dot(x),phi,dot(phi)
R = 0.001;
% 计算出k
K =lqr(cart,Q,R);
% 计算Acl的特征值，观察系统稳定性
eig(A-B*K)
%% 求解微分方程
tspan=0:0.05:10;
y_0=[0;0;pi/12;0];
[t,y] = ode45(@(t,y)solve_cart(t,y,m,M,L,g,J,K),tspan, y_0);

figure;
grid on;
hold on;
plot(tspan,y(:,1));
plot(tspan,y(:,2));
plot(tspan,y(:,3));
plot(tspan,y(:,4));
legend({'x', '$$\dot{x}$$', '$$\varphi$$', '$$\dot{\varphi}$$'}, 'Interpreter', 'latex');
hold off
K
