clear
clc

global error_threshold;
error_threshold = 1e-3;

addpath(genpath(pwd));

% Read config:
% Please fully define "config_modeling.yaml" (necessary) and "config_manual_robot.yaml" (optional).
% Please fully define "config_workspace.yaml" (necessary).
config = fun_read_config();

if config.workspace.use_parallel
    disp('Use parallel computation.')

    if isempty(gcp('nocreate'))
        parpool;
    end

end

% Build robot model
[robot, config] = fun_build_robot(config);

% Generate sample points
samples = fun_generate_workspace_samples(config.workspace);

% Analyze workspace
workspace_data = fun_workspace_analysis(robot, samples, config);

% Write data to file for next use
if ~exist('data/workspace', 'dir')
    mkdir('data/workspace');
end

% The same file is used by the visualization script; it holds the results of the
% currently active scheme and is overwritten by every run.
filename = 'data/workspace/workspace_ik.mat';
save(filename, 'config', 'samples', 'workspace_data');
disp(['Write the workspace data to file "', filename, '".']);
