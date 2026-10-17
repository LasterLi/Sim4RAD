clear
clc

addpath(genpath(pwd));

% Read config
% Please fully define "config_modeling.yaml" (necessary) and "config_manual_robot.yaml" (optional).
config = fun_read_config();

% Build robot model
[robot, config] = fun_build_robot(config);

q = homeConfiguration(robot);
Num_panel = ceil(length(q) / 2);

disp(['Plot the robot to test joint motion one by one.']);

figure('Name', 'Joint = +pi/6');
for i = 1:length(q)
    subplot(2, Num_panel, i);
    q = homeConfiguration(robot);
    q(i) = q(i) + pi / 6;
    fun_show_robot(robot, q, config);
    title(['Joint ', num2str(i), ' = +pi/6.']);
end

% Joint lower limits
figure('Name', 'Joint at lower limit');
for i = 1:length(q)
    subplot(2, Num_panel, i);
    q = homeConfiguration(robot);
    q(i) = config.modeling.joint_lower_limit(i);
    fun_show_robot(robot, q, config);
    title(['Joint ', num2str(i), ' = lower limit.']);
end

% Joint upper limits
figure('Name', 'Joint at upper limit');
for i = 1:length(q)
    subplot(2, Num_panel, i);
    q = homeConfiguration(robot);
    q(i) = config.modeling.joint_upper_limit(i);
    fun_show_robot(robot, q, config);
    title(['Joint ', num2str(i), ' = upper limit.']);
end
