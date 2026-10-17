function fun_plot_workspace(workspace_data, config)

    base_rotm = quat2rotm(config.base_orientation);
    base_tform = [base_rotm, config.base_position(:); 0, 0, 0, 1];

    % dexterity pionts
    fun_plot_workspace_dexterity(workspace_data, base_tform, config.position_resolution);

    % sphere figure
    % fun_plot_workspace_point_cloud(workspace_data, config.position_resolution, config.vis_resolution_factor, base_tform);
