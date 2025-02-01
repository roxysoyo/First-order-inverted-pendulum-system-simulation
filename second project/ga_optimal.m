clc;
clear;

% 设置优化参数范围 (增益值范围)
lb = [2, 2];  % 下界：增益最小值
ub = [6, 6];  % 上界：增益最大值

% 定义遗传算法选项
options = optimoptions('ga', ...
    'PopulationSize', 20, ...          % 种群大小
    'MaxGenerations', 20, ...          % 最大代数
    'Display', 'iter', ...             % 显示每代优化信息
    'FunctionTolerance', 5e-1, ...     % 目标函数容忍度
    'PlotFcn', @gaplotbestf);          % 绘制最优解收敛图

% 使用遗传算法优化增益
[x_opt, fval] = ga(@fitness_function, 2, [], [], [], [], lb, ub, [], options);

% 输出最优增益值
fprintf('最优增益值：Angle_Gain = %.4f, Position_Gain = %.4f\n', x_opt(1), x_opt(2));
fprintf('到达目标位置的最短时间：%.4f 秒\n', fval);

% -------------------------
% 定义目标函数
function time_to_target = fitness_function(gains)
    % gains: [Angle_Gain, Position_Gain]
    Angle_Gain = gains(1);
    Position_Gain = gains(2);

    % 设置模型参数
    set_param('pendulum/Angle_Gain', 'Gain', num2str(Angle_Gain));
    set_param('pendulum/Position_Gain', 'Gain', num2str(Position_Gain));

    % 运行仿真
    out = sim('pendulum', 'StopTime', '800');

    % 提取输出变量
    time = out.x_t.Time;
    value = out.x_t.Data;

    % 提取位移数据
    position = value(:, 1);

    % 检查是否有超调量
    place = 1000;
    overshoot_penalty = 0; % 超调惩罚项
    if max(position) > place % 超过目标值
        overshoot_penalty = 1e3 * (max(position) - place); % 超调越多，惩罚越大
    end

    % 找到第一次达到目标位置的时间
    i = find(position >= 999, 1); % 找到首次达到目标的索引
    if ~isempty(i)
        time_to_target = time(i) + overshoot_penalty; % 加入超调惩罚
    else
        time_to_target = 1e3; % 仿真失败或无法找到插入点时返回较大值
    end
end
