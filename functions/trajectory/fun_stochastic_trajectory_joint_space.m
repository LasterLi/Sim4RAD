function trajectory_samples = fun_stochastic_trajectory_joint_space(robot, num_samples, min_ee_distance, joint_lower_limits, joint_upper_limits, dt, interpolation_method)
    dof = length(joint_lower_limits);

    trajectory_samples = struct;
    t0 = datetime('now');

    for i = 1:num_samples

        while true
            q1 = rand(1, dof) .* (joint_upper_limits - joint_lower_limits) + joint_lower_limits;
            q2 = rand(1, dof) .* (joint_upper_limits - joint_lower_limits) + joint_lower_limits;

            pos1 = fun_fk(robot, q1);
            pos2 = fun_fk(robot, q2);

            dis = norm(pos1(1:3) - pos2(1:3));

            if dis >= min_ee_distance
                break;
            end

        end

        [t, q, qd, qdd] = fun_joint_interpolation(q1, zeros(1, dof), q2, zeros(1, dof), 2, dt, interpolation_method);

        trajectory_samples(i).t = t;
        trajectory_samples(i).q = q;
        trajectory_samples(i).qd = qd;
        trajectory_samples(i).qdd = qdd;
        trajectory_samples(i).dis = dis;

        fun_for_progress(num_samples, i, t0);
    end
