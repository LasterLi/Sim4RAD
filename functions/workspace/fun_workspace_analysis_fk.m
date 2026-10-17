function workspace_data = fun_workspace_analysis_fk(robot, fk_sampling_num, workspace_data, samples, ...
        joint_lower_limits, joint_upper_limits, space_range, use_parallel, consider_self_collision)

    dof = length(homeConfiguration(robot));

    position_resolution = norm(workspace_data(1).position - workspace_data(2).position);

    disp('Randomly generate configurations, and perform forward kinematics.');

    num_group = 1000;
    num_iter = fk_sampling_num / num_group;

    q_samples = struct;

    if use_parallel
        t0 = datetime('now');
        fun_parfor_progress(num_iter, 0, t0);

        parfor i = 1:num_iter
            q_samples(i).data = fun_workspace_fk_sampling(robot, joint_upper_limits, joint_lower_limits, space_range, samples, position_resolution, num_group, consider_self_collision);
            fun_parfor_progress(num_iter, i, t0);
        end

        fun_parfor_progress(num_iter, -1, t0);
    else
        t0 = datetime('now');

        for i = 1:num_iter
            q_samples(i).data = fun_workspace_fk_sampling(robot, joint_upper_limits, joint_lower_limits, space_range, samples, position_resolution, num_group, consider_self_collision);
            fun_for_progress(num_iter, i, t0);
        end

    end

    t0 = datetime('now');

    for i = 1:num_iter
        workspace_data = fun_workspace_fk_samples_mapping(workspace_data, q_samples(i).data);
        fun_for_progress(num_iter, i, t0);
    end

end
