function workspace_data = fun_workspace_fk_samples_mapping(workspace_data, q_samples)

    q = q_samples(:, 1:(end - 3));
    pos_idx = q_samples(:, end - 2);
    dir_idx = q_samples(:, end - 1);
    singularity = q_samples(:, end);

    for i = 1:length(pos_idx)

        if dir_idx(i) > 0
            workspace_data(pos_idx(i)).reachability = 1;

            if ~workspace_data(pos_idx(i)).pose_reachability(dir_idx(i)).reachability
                workspace_data(pos_idx(i)).pose_reachability(dir_idx(i)).reachability = 1;
                workspace_data(pos_idx(i)).pose_reachability(dir_idx(i)).q = q(i, :);
                workspace_data(pos_idx(i)).pose_reachability(dir_idx(i)).singularity = singularity(i);
            else

                if singularity(i) > workspace_data(pos_idx(i)).pose_reachability(dir_idx(i)).singularity
                    workspace_data(pos_idx(i)).pose_reachability(dir_idx(i)).q = q(i, :);
                    workspace_data(pos_idx(i)).pose_reachability(dir_idx(i)).singularity = singularity(i);
                end

            end

        end

    end

end
