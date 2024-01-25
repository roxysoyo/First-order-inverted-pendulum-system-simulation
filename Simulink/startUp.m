clear
%% cart properties
global M m g L J K_LQR
M = 0.2275;
m = 0.0923;
g = 9.8;
L = 0.185;
J = 0.00412;

%% cart initial conditions
x_0 = 0;
y_0 = 0.08;
phi_0 = 20;

%% controllers
LQR = 1;

if LQR
    K_LQR = cartLQR;
end