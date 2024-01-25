function K=cartLQR
%% parameters
global M m g L J
% M = 0.2275;
% m = 0.0923;
% g = 9.8;
% L = 0.185;
% J = 0.00412;
a=J/(m*L);
fen_mu=M*L+M*a+m*a;
%% Define system matrices
A = [
    0   1            0               0;
    0   0        m*g*L/fen_mu        0;
    0   0            0               1;
    0   0       (m+M)*g/fen_mu       0
];
B = [0; (L+a)/fen_mu; 0; 1/fen_mu];
C = eye(4);
D = 0;
%% Create state-space system
cart = ss(A, B, C, D);
%% LQR design
Q = diag([10 1 10 1]); % x, dot(x), phi, dot(phi)
R = 0.01;
%% Compute LQR gains
K = lqr(cart, Q, R);
end