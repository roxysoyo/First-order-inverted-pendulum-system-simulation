% 加载两个数据文件
data1 = load('trimf.mat').data;
data2 = load('gauss.mat').data;

% 提取各文件中的变量（假设列顺序一致）
time1 = data1(:, 1); x1 = data1(:, 2); v1 = data1(:, 3); phi1 = data1(:, 4); omega1 = data1(:, 5); F1 = data1(:, 6);
time2 = data2(:, 1); x2 = data2(:, 2); v2 = data2(:, 3); phi2 = data2(:, 4); omega2 = data2(:, 5); F2 = data2(:, 6);

% 图1: x (位移) 对比
figure;
plot(time1, x1, 'r', 'LineWidth', 1.5); hold on;
plot(time2, x2, 'Color', [0.5, 0.7, 1], 'LineWidth', 1.5);
title('三角隶属度函数与高斯隶属度函数的位移x对比');
xlabel('时间t (s)');
ylabel('位移x (m)');
legend('trimf', 'gauss');
grid on;

% 图2: v (速度) 对比
figure;
plot(time1, v1, 'r', 'LineWidth', 1.5); hold on;
plot(time2, v2, 'Color', [0.5, 0.7, 1], 'LineWidth', 1.5);
title('三角隶属度函数与高斯隶属度函数的速度v对比');
xlabel('时间t (s)');
ylabel('速度v (m/s)');
legend('trimf', 'gauss');
grid on;

% 图3: phi (角度) 对比
figure;
plot(time1, phi1, 'r', 'LineWidth', 1.5);
hold on;
plot(time2, phi2, 'Color', [0.5, 0.7, 1], 'LineWidth', 1.5);
title('三角隶属度函数与高斯隶属度函数的角度 \phi 对比');
xlabel('时间t (s)');
ylabel('角度\phi (rad)');
legend('trimf', 'gauss');
grid on;

% 图4: omega (角速度) 对比
figure;
plot(time1, omega1, 'r', 'LineWidth', 1.5); hold on;
plot(time2, omega2, 'Color', [0.5, 0.7, 1], 'LineWidth', 1.5);
title('三角隶属度函数与高斯隶属度函数的角速度 \omega 对比');
xlabel('时间t (s)');
ylabel('角速度\omega (rad/s)');
legend('trimf', 'gauss');
grid on;

% 图5: F (力) 对比
figure;
plot(time1, F1, 'r', 'LineWidth', 1.5); hold on;
plot(time2, F2, 'Color', [0.5, 0.7, 1], 'LineWidth', 1.5);
title('三角隶属度函数与高斯隶属度函数的力 F 对比');
xlabel('时间t (s)');
ylabel('力F (N)');
legend('trimf', 'gauss');
grid on;
