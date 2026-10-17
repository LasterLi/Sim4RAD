function workspace_data = fun_init_workspace_data(samples, dof)

    disp('Initialize the workspace data structure.')
    workspace_data = struct;
    t0 = datetime('now');

    for i = 1:samples.num_pos

        workspace_data(i).position = samples.points(:, i);
        workspace_data(i).reachability = 0;
        workspace_data(i).dexterity = 0;
        workspace_data(i).center_direction = zeros(3, 1);

        pose_reachability = struct;

        for j = 1:samples.num_dir
            pose_reachability(j).direction = samples.directions(:, j);
            pose_reachability(j).reachability = 0;
            pose_reachability(j).q = zeros(1, dof);
            pose_reachability(j).singularity = 0;
        end

        workspace_data(i).pose_reachability = pose_reachability;
        fun_for_progress(samples.num_pos, i, t0);
    end

end
