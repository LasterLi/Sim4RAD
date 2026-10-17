clear
clc

addpath(genpath(pwd));

config = fun_read_config();

% Build robot model
robot = fun_build_robot(config);

% 按摩轨迹生成

x = 0.25;
y = -0.2;
z = -0.35;

quat = eul2quat([0, pi / 2, pi / 2]);

[q, ~, flag] = fun_ik(robot, [x, y, z, quat], [], zeros(1, 7), 0.001);

dt = config.dynamics.time_step;

t = 0:dt:0.6;
theta = linspace(0, 2 * pi, length(t));
x = x + 0.05 * cos(theta);
y = y + 0.05 * sin(theta);
z = z * ones(1, length(t));

plot3(x, y, z)
axis equal

quat = quat .* ones(length(t), 4);
data = [t', x', y', z', quat];
data5 = [];

for i = 1:5
    data_i = data(1:end - 1, :);
    data_i(:, 1) = data_i(:, 1) + (i - 1) * 0.6;
    data5 = [data5; data_i];
end

data5 = [data5; [3, data(end, 2:end)]];

if ~exist('data/trajectory', 'dir')
    mkdir('data/trajectory');
end

csvwrite('data/trajectory/an_mo.csv', data5);
disp(['Trajectory data is written to "data/trajectory/an_mo.csv".']);
