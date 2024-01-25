function Fx = LQRControl(states)
global K_LQR
%% desired value
X_des = [0;0;0;0];
%% control law
Fx = K_LQR*(X_des - states);
end