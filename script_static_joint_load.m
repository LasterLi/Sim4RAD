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

num_samples = config.modeling.num_sampling;
disp(['Stochastically generate ', num2str(num_samples), ' configurations, and calculate the static loads of joints ... ']);
[q, arm_of_force, tau] = fun_static_sampling(robot, config.modeling.joint_lower_limit, config.modeling.joint_upper_limit, num_samples);

disp('Plot the results: arm_of_force vs joint torque.')
fun_plot_static_arm_of_force_joint_effort(arm_of_force, tau, 0.4)
