function trajectory_samples = fun_stochastic_trajectory_fk(robot, trajectory_samples, use_parallel)

    N = length(trajectory_samples);
    disp('Update the end-effector trajectory data.')

    if use_parallel
        t0 = datetime('now');
        fun_parfor_progress(N, 0, t0);

        parfor i = 1:N
            [trajectory_samples(i).pos, trajectory_samples(i).vel, trajectory_samples(i).acc, ...
                 trajectory_samples(i).linear_vel, trajectory_samples(i).ang_vel, ...
                 trajectory_samples(i).linear_acc, trajectory_samples(i).ang_acc] = ...
                fun_trajectory_fk(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                trajectory_samples(i).qd, trajectory_samples(i).qdd);

            fun_parfor_progress(N, i, t0);
        end

        fun_parfor_progress(N, -1, t0);
    else
        t0 = datetime('now');

        for i = 1:N
            [trajectory_samples(i).pos, trajectory_samples(i).vel, trajectory_samples(i).acc, ...
                 trajectory_samples(i).linear_vel, trajectory_samples(i).ang_vel, ...
                 trajectory_samples(i).linear_acc, trajectory_samples(i).ang_acc] = ...
                fun_trajectory_fk(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                trajectory_samples(i).qd, trajectory_samples(i).qdd);

            fun_for_progress(N, i, t0);
        end

    end

end
