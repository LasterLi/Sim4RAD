function config = fun_read_config()
    % Read all configuration files into a struct.
    % The config directory is resolved relative to this file, so that the entry
    % scripts can be started from any working directory.

    config = struct;

    project_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    config_dir = fullfile(project_root, 'config');

    if ~exist(fullfile(config_dir, 'config_modeling.yaml'), 'file')
        config_dir = 'config';   % fallback: relative to the working directory
    end

    config.modeling = fun_read_yaml(fullfile(config_dir, 'config_modeling.yaml'));
    config.modeling.joint_lower_limit = config.modeling.joint_lower_limit * pi / 180;
    config.modeling.joint_upper_limit = config.modeling.joint_upper_limit * pi / 180;

    if (~config.modeling.using_urdf_model)
        config.manual_robot = fun_read_yaml(fullfile(config_dir, 'config_manual_robot.yaml'));
    end

    config.dynamics = fun_read_yaml(fullfile(config_dir, 'config_dynamics.yaml'));

    config.stochastic_dynamics = fun_read_yaml(fullfile(config_dir, 'config_stochastic_dynamics.yaml'));

    config.workspace = fun_read_yaml(fullfile(config_dir, 'config_workspace.yaml'));

end
