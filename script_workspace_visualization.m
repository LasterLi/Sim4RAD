clear
clc

global error_threshold;
error_threshold = 1e-3;

addpath(genpath(pwd));

filename = 'data/workspace/workspace_ik.mat';
disp(['Read the workspace data from file "', filename, '".']);
fileInfo = dir(filename);
fileSizeBytes = fileInfo.bytes;
fileSizeMB = fileSizeBytes / (1024 * 1024); % 转换为 MB
disp(['File size: ', num2str(fileSizeMB), ' MB. Need time: about ', num2str(fileSizeMB / 25), ' s.']);
data = load(filename, 'config', 'samples', 'workspace_data');
config = data.config;
samples = data.samples;
workspace_data = data.workspace_data;

% config = fun_read_config();

% Build robot model
robot = fun_build_robot(config);

% clear the workspace_data
workspace_data_cleaned = fun_clean_workspace_data(workspace_data, config.workspace);

% plot workspace_data.
fun_plot_workspace(workspace_data_cleaned, config.workspace);
