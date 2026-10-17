function trajectory_samples = fun_stochastic_trajectory_inverse_dynamics(robot, trajectory_samples, transmission, use_parallel);

    N = length(trajectory_samples);
    disp('Update the trajectory dynamics data.')

    if use_parallel
        t0 = datetime('now');
        fun_parfor_progress(N, 0, t0);

        parfor i = 1:N

            [trajectory_samples(i).tau_motor, trajectory_samples(i).tau_out, ...
                 trajectory_samples(i).power_motor, trajectory_samples(i).power_out] = ...
                fun_trajectory_inverse_dynamics(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                trajectory_samples(i).qd, trajectory_samples(i).qdd, transmission, zeros(1, 6));

            fun_parfor_progress(N, i, t0);
        end

        fun_parfor_progress(N, -1, t0);
    else
        t0 = datetime('now');

        for i = 1:N
            [trajectory_samples(i).tau_motor, trajectory_samples(i).tau_out, ...
                 trajectory_samples(i).power_motor, trajectory_samples(i).power_out] = ...
                fun_trajectory_inverse_dynamics(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                trajectory_samples(i).qd, trajectory_samples(i).qdd, transmission, zeros(1, 6));

            fun_for_progress(N, i, t0);
        end

    end

    disp('----------------------------------------------------------------------------------------------------');

end
