clear
clc

addpath(genpath(pwd));

% Read config:
% Please fully define "config_modeling.yaml" (necessary) and "config_manual_robot.yaml" (optional).
config = fun_read_config();

% Build robot model
[robot, config] = fun_build_robot(config);

if config.modeling.gravity_direction == 0
    disp('No gravity! So static load is null.')
    return
end

% List all orthogonal poses.
disp(['Searching all orthogonal configurations that may have the max static loads of joints ...']);
[tau_max, q_all, tau_all] = fun_search_static_load(robot, config);
disp(['Max static load of joints are: ', num2str(tau_max)]);

disp('----------------------------------------------------------------------------------------------------');
disp(['Merging the similar configurations, finding the typical configurations ...']);
[q_typical, tau_typical, max_joint_id] = fun_typical_static_config(q_all, tau_all);

% plot typical poses
Num_panel = ceil(size(q_typical, 1) / 2);

for i = 1:size(q_typical, 1)
    subplot(2, Num_panel, i);
    fun_show_robot(robot, q_typical(i, :), config);
    panel_title = ['Joint ', num2str(find(max_joint_id(i, :) == 1))];
    title([panel_title, ' = max load']);
end

disp(['Typical poses of max static loads have been ploted.']);

for i = 1:size(q_typical, 1)
    disp(['Typical pose ', num2str(i), ':']);
    disp(['Joint position: ', num2str(q_typical(i, :))]);
    disp(['Joint torque: ', num2str(tau_typical(i, :))]);
end
