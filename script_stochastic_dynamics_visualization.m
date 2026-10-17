clear
clc

addpath(genpath(pwd));

filename = 'data/stochastic_dynamics/stoch_dyn.mat';
data = load(filename, 'config', 'trajectory_samples');
% Use the config that was saved together with the data, so that the robot model
% matches the one used to generate the results.
config = data.config;
trajectory_samples = data.trajectory_samples;

% Build robot model
robot = fun_build_robot(config);

% plot simulation results.
[transmission, joint_capability] = fun_get_joint_dynamics_capability(config.dynamics);
statistics = fun_statistics(trajectory_samples, transmission);
fun_plot_statistics(statistics, joint_capability);

fun_plot_statistics_joint_effort(statistics, joint_capability, 0.2);

% fun_plot_statistics_joint_vel_effort(trajectory_samples, joint_capability, transmission);
