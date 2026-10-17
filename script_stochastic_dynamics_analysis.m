clear
clc

global error_threshold;
error_threshold = 1e-3;

addpath(genpath(pwd));

% Read config:
% Please fully define "config_modeling.yaml" (necessary) and "config_manual_robot.yaml" (optional).
% Please fully define "config_dynamics.yaml" (necessary).
% Please fully define "config_stochastic_dynamics.yaml" (necessary).
config = fun_read_config();

if config.stochastic_dynamics.use_parallel
    disp('Use parallel computation.')

    if isempty(gcp('nocreate'))
        parpool;
    end

end

% Build robot model
[robot, config] = fun_build_robot(config);

% Generate stochastic trajectory
trajectory_samples = fun_stochastic_trajectory(robot, config);

% Update the time scale factor to satisfy the trajectory constraint.
trajectory_samples = fun_stochastic_trajectory_constraint(robot, trajectory_samples, config);

% Calculate the end effector trajectory
trajectory_samples = fun_stochastic_trajectory_fk(robot, trajectory_samples, config.stochastic_dynamics.use_parallel);

% Get the joint drive requirement by dynamics simulation
if config.dynamics.consider_transmission_loss
    disp('Transmission loss is considered!');
end

[transmission, joint_capability] = fun_get_joint_dynamics_capability(config.dynamics);
trajectory_samples = fun_stochastic_trajectory_inverse_dynamics(robot, trajectory_samples, transmission, ...
    config.stochastic_dynamics.use_parallel);

% Write data to file for next use
if ~exist('data/stochastic_dynamics', 'dir')
    mkdir('data/stochastic_dynamics');
end

% The same file is used by the visualization script; it holds the results of the
% currently active scheme and is overwritten by every run.
filename = 'data/stochastic_dynamics/stoch_dyn.mat';
save(filename, 'config', 'trajectory_samples');
disp(['Write the stochastic dynamics data to file "', filename, '".']);
