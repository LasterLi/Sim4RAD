function workspace_data = fun_workspace_analysis_ik(robot, workspace_data, joint_lower_limits, joint_upper_limits, use_parallel, consider_self_collision)

    % joint_limits = [joint_lower_limits; joint_upper_limits]';
    q_init = (joint_lower_limits + joint_upper_limits) / 2;

    num_pos = length(workspace_data);
    num_dir = length(workspace_data(1).pose_reachability);

    gik = fun_ik_constraint_construct(robot);
    LastLink = robot.BodyNames{1, end};

    if use_parallel
        t0 = datetime('now');
        fun_parfor_progress(num_pos, 0, t0);

        parfor i = 1:num_pos
            pos = workspace_data(i).position;

            for j = 1:num_dir
                direction = workspace_data(i).pose_reachability(j).direction;
                [q, flag] = fun_ik_constraint(gik, LastLink, pos', direction', q_init, 0.01);

                if consider_self_collision

                    if checkCollision(robot, q)
                        flag = 0;
                    end

                end

                if flag ~= 0
                    workspace_data(i).reachability = 1;
                    workspace_data(i).pose_reachability(j).reachability = 1;
                    workspace_data(i).pose_reachability(j).q = q;
                    workspace_data(i).pose_reachability(j).singularity = fun_singularity(robot, q);
                end

            end

            fun_parfor_progress(num_pos, i, t0);
        end

        fun_parfor_progress(num_pos, -1, t0);
    else
        t0 = datetime('now');

        for i = 1:num_pos
            pos = workspace_data(i).position;

            for j = 1:num_dir
                direction = workspace_data(i).pose_reachability(j).direction;
                [q, flag] = fun_ik_constraint(gik, LastLink, pos', direction', q_init, 0.01);

                if consider_self_collision

                    if checkCollision(robot, q)
                        flag = 0;
                    end

                end

                if flag ~= 0
                    workspace_data(i).reachability = 1;
                    workspace_data(i).pose_reachability(j).reachability = 1;
                    workspace_data(i).pose_reachability(j).q = q;
                    workspace_data(i).pose_reachability(j).singularity = fun_singularity(robot, q);
                end

            end

            fun_for_progress(num_pos, i, t0);
        end

    end
