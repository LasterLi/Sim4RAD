function workspace_data = fun_clean_workspace_data(workspace_data, config)

    disp('Clean the workspace_data.')

    num_pos = length(workspace_data);
    num_dir = length(workspace_data(1).pose_reachability);

    position_resolution = config.position_resolution;

    use_vis_custom_bound = config.use_vis_custom_bound;
    custom_vis_lower_bound = config.custom_vis_lower_bound;
    custom_vis_upper_bound = config.custom_vis_upper_bound;
    vis_resolution_factor = config.vis_resolution_factor;

    keep_idx = ones(1, num_pos);

    % t0 = datetime('now');

    for i = 1:num_pos

        if ~workspace_data(i).reachability
            keep_idx(i) = 0;
            continue;
        end

        if use_vis_custom_bound

            if ~all(workspace_data(i).position <= custom_vis_upper_bound(:)) || ~all(workspace_data(i).position >= custom_vis_lower_bound(:))

                keep_idx(i) = 0;
                continue;
            end

        end

        if vis_resolution_factor > 1
            vis_resolution = vis_resolution_factor * position_resolution;
            p_mod = mod(workspace_data(i).position, vis_resolution);
            p_mod = min(p_mod, abs(p_mod - vis_resolution));

            if ~all(p_mod < 1e-3)
                keep_idx(i) = 0;
                continue;
            end

        end

        % fun_for_progress(num_pos, i, t0);
    end

    workspace_data(keep_idx == 0) = [];

end
