function q_samples = fun_workspace_fk_sampling(robot, joint_upper_limits, joint_lower_limits, space_range, samples, position_resolution, num_group, consider_self_collision)
    dof = length(homeConfiguration(robot));

    if consider_self_collision
        q = rand(num_group, dof) .* (joint_upper_limits - joint_lower_limits) + joint_lower_limits;

        for i = 1:num_group
            is_collision = checkCollision(robot, q(i, :))

            while is_collision
                q(i, :) = rand(1, dof) .* (joint_upper_limits - joint_lower_limits) + joint_lower_limits;
                is_collision = checkCollision(robot, q(i, :))
            end

        end

    else
        q = rand(num_group, dof) .* (joint_upper_limits - joint_lower_limits) + joint_lower_limits;
    end

    pos_idx = zeros(num_group, 1);
    dir_idx = zeros(num_group, 1);
    singularity = zeros(num_group, 1);

    for i = 1:num_group
        p = fun_fk(robot, q(i, :));
        pos = p(1:3);

        if all(pos(:) <= space_range.upper(:)) && all(pos(:) >= space_range.lower(:))

            [pos_idx(i), dis] = fun_find_idx(samples.points, pos(:));

            if dis < position_resolution / 2

                quat = p(4:7);
                R = quat2rotm(quat);
                dir = R(:, 3);

                [dir_idx(i), dir_dis] = fun_find_idx(samples.directions, dir(:));

                singularity(i) = fun_singularity(robot, q(i, :));
            end

        end

    end

    q = q(dir_idx > 0, :);
    pos_idx = pos_idx(dir_idx > 0);
    dir_idx = dir_idx(dir_idx > 0);
    singularity = singularity(dir_idx > 0);

    q_samples = [q, pos_idx, dir_idx, singularity];
end
