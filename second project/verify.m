clc;
clear;
% pendulum.slx模型中所有的Gain模块
GainCell = find_system('pendulum','BlockType','Gain');

Angle_Gain = GainCell{1};
Position_Gain = GainCell{2};

set_param(Angle_Gain, 'Gain', '5.991'); 
set_param(Position_Gain, 'Gain', '5.991'); 

out = sim('pendulum', 'StopTime', '1000');  % 仿真1000秒

time = out.x.time;
x_value = out.x.signals.values;
v_value = out.v.signals.values;
phi_value = out.phi.signals.values;
omega_value = out.omega.signals.values;
F_value = out.F.signals.values;

data = [time, x_value, v_value, phi_value, omega_value, F_value];

% 保存到 MAT 文件中
% save('trimf.mat', 'data');

% 找到第一次 place 能插入的位置
i = -1; % 默认值，表示未找到
place = 998;
for idx = 1:length(x_value)-1
    if x_value(idx) <= place && x_value(idx + 1) > place
        i = idx;
        break;
    end
end

% 输出结果
if i ~= -1
    fprintf('能够插入的位置是第 %d 个和第 %d 个序列之间\n', i, i+1);
else
    fprintf('无法插入到序列中\n');
end

t_min = time(i);
