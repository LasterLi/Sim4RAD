function workspace_data = fun_update_workspace_data(workspace_data)
    num_pos = length(workspace_data);
    num_dir = length(workspace_data(1).pose_reachability);
    disp('Update the dexterity and center_direction of workspace_data.')

    pos_reachability = zeros(1, num_pos);

    t0 = datetime('now');

    for i = 1:num_pos

        if workspace_data(i).reachability
            a = 0;
            center_dir = zeros(3, 1);

            for j = 1:num_dir

                if workspace_data(i).pose_reachability(j).reachability
                    a = a + 1;
                    center_dir = center_dir + workspace_data(i).pose_reachability(j).direction;
                end

            end

            dexterity = a / num_dir;

            if dexterity < 1
                center_dir = center_dir / a;
                workspace_data(i).center_direction = center_dir / norm(center_dir);
            end

            workspace_data(i).dexterity = dexterity;

            pos_reachability(i) = 1;
        end

        fun_for_progress(num_pos, i, t0);
    end

    workspace_data(pos_reachability == 0) = [];
end
