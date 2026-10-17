function workspace_data = fun_workspace_analysis(robot, samples, config)

    dof = length(homeConfiguration(robot));

    if config.workspace.consider_self_collision
        % initial check the collision
        % If it is in collision at the homeConfiguration, the collision geometries may be set wrongly.
        q0 = homeConfiguration(robot);
        [is_collision, separation_dist] = checkCollision(robot, q0);

        if is_collision
            [row, col] = find(isnan(separation_dist));

            for i = 1:length(row)
                warning(['WARNING: it is in collision between ', num2str(row(i)), ' and ', num2str(col(i)), ' at home configuration, Please check!']);
            end

        end

    end

    workspace_data = fun_init_workspace_data(samples, dof);

    joint_lower_limits = config.modeling.joint_lower_limit;
    joint_upper_limits = config.modeling.joint_upper_limit;

    if config.workspace.use_ik
        disp('Use ik to explore the workspace.');
        workspace_data = fun_workspace_analysis_ik(robot, workspace_data, joint_lower_limits, joint_upper_limits, ...
            config.workspace.use_parallel, config.workspace.consider_self_collision);
    else
        disp('Use fk to explore the workspace.');
        space_range = fun_get_space_range(config.workspace);
        workspace_data = fun_workspace_analysis_fk(robot, config.workspace.fk_sampling_num, workspace_data, samples, ...
            joint_lower_limits, joint_upper_limits, space_range, config.workspace.use_parallel, ...
            config.workspace.consider_self_collision);
    end

    workspace_data = fun_update_workspace_data(workspace_data);

end
