function dydt = solve_cart(t, y, m, M, L, g, J, K)
    %% Inputs:
    % t: 时间
    % y: 状态向量 [x; x_dot; phi; phi_dot]
    % M: 小车质量
    % m: 摆杆质量
    % L: 摆杆长度
    % J: 摆杆绕质心的转动惯量
    % g: 重力加速度
    % K: 控制器增益矩阵

    %% Unpack 状态向量
    % x = y(1);
    % x_dot = y(2);
    % phi = y(3);
    % phi_dot = y(4);

    %% Calculate 动态方程
    dydt = zeros(4, 1);

    %% 主要的动态方程
    a = J / (m * L);
    fen_mu = M * L + M * a + m * a;
    % x=x1;dot(x)=x2;phi=x3;dot(phi)=x4;
    % (M+m)*dot(x2)-m*l*dot(x4)=F;
    % l*dot(x4)-dot(x2)+(J/(m*l))*dot(x4)-g*x3=0
    
    dydt(1) = y(2);
    dydt(2) = (1/fen_mu) * (m * g * L * y(3) + (L + a) * (-K * (y - [0; 0; 0; 0])));
    dydt(3) = y(4);
    dydt(4) = (1/fen_mu) * ((M + m) * g * y(3) - K * (y - [0; 0; 0; 0]));

end
