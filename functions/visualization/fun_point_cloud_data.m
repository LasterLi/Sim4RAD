function point_cloud_data = fun_point_cloud_data(workspace_data, vis_sphere_size, base_tform)

    disp('Generate point cloud data.');
    num_pos = length(workspace_data);
    num_dir = length(workspace_data(1).pose_reachability);

    point_cloud_data = zeros(num_pos * num_dir, 4);

    % t0 = datetime('now');

    for i = 1:num_pos
        point_data = zeros(num_dir, 4);

        for j = 1:num_dir
            point_data(j, 1:3) = workspace_data(i).position + vis_sphere_size * workspace_data(i).pose_reachability(j).direction;

            if workspace_data(i).pose_reachability(j).reachability
                point_data(j, 4) = 1;
            end

        end

        point_cloud_data(((i - 1) * num_dir + 1):(i * num_dir), :) = point_data;
        % fun_for_progress(num_pos, i, t0);

    end

    base_pos = base_tform(1:3, 4)';
    base_rotm = base_tform(1:3, 1:3);
    point_cloud_data(:, 1:3) = (base_rotm * point_cloud_data(:, 1:3)')' + base_pos;
