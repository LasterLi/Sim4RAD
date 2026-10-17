function trajectory_samples = fun_stochastic_trajectory_constraint(robot, trajectory_samples, config);

    N = length(trajectory_samples);

    if config.dynamics.constraint_mode == 1
        duration = config.dynamics.trajectory_duration;
        disp(['trajectory_constraint is duration = ', num2str(duration), ' s']);

        if config.stochastic_dynamics.use_parallel
            t0 = datetime('now');
            fun_parfor_progress(N, 0, t0);

            parfor i = 1:N
                [trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd] = ...
                    fun_trajectory_scale(trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd, duration);
                fun_parfor_progress(N, i, t0);
            end

            fun_parfor_progress(N, -1, t0);
        else
            t0 = datetime('now');

            for i = 1:N
                [trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd] = ...
                    fun_trajectory_scale(trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd, duration);
                fun_for_progress(N, i, t0);
            end

        end

    elseif config.dynamics.constraint_mode == 2
        max_ee_velocity = config.dynamics.max_ee_velocity;
        disp(['trajectory_constraint is max_ee_velocity = ', num2str(max_ee_velocity), ' m/s']);

        if config.stochastic_dynamics.use_parallel
            t0 = datetime('now');
            fun_parfor_progress(N, 0, t0);

            parfor i = 1:N
                [trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd] = ...
                    fun_trajectory_constraint_max_ee_velocity(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                    trajectory_samples(i).qd, trajectory_samples(i).qdd, max_ee_velocity);
                fun_parfor_progress(N, i, t0);
            end

            fun_parfor_progress(N, -1, t0);
        else
            t0 = datetime('now');

            for i = 1:N
                [trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd] = ...
                    fun_trajectory_constraint_max_ee_velocity(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                    trajectory_samples(i).qd, trajectory_samples(i).qdd, max_ee_velocity);
                fun_for_progress(N, i, t0);
            end

        end

    elseif config.dynamics.constraint_mode == 3
        max_ee_acceleration = config.dynamics.max_ee_acceleration;
        disp(['trajectory_constraint is max_ee_acceleration = ', num2str(max_ee_acceleration), ' m/s^2']);

        if config.stochastic_dynamics.use_parallel
            t0 = datetime('now');
            fun_parfor_progress(N, 0, t0);

            parfor i = 1:N
                [trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd] = ...
                    fun_trajectory_constraint_max_ee_acceleration(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                    trajectory_samples(i).qd, trajectory_samples(i).qdd, max_ee_acceleration);
                fun_parfor_progress(N, i, t0);
            end

            fun_parfor_progress(N, -1, t0);
        else
            t0 = datetime('now');

            for i = 1:N
                [trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd] = ...
                    fun_trajectory_constraint_max_ee_acceleration(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                    trajectory_samples(i).qd, trajectory_samples(i).qdd, max_ee_acceleration);
                fun_for_progress(N, i, t0);
            end

        end

    elseif config.dynamics.constraint_mode == 4
        [transmission, joint_capability] = fun_get_joint_dynamics_capability(config.dynamics);
        disp('trajectory_constraint is joint_capability.');
        disp(['Maximal joint torque is: ', num2str(config.dynamics.joint_peak_torque)]);
        disp(['Maximal joint velocity is: ', num2str(config.dynamics.joint_peak_velocity)]);

        if config.stochastic_dynamics.use_parallel
            t0 = datetime('now');
            fun_parfor_progress(N, 0, t0);

            parfor i = 1:N
                [trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd] = ...
                    fun_trajectory_constraint_actuator_limit(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                    trajectory_samples(i).qd, trajectory_samples(i).qdd, transmission, joint_capability);
                fun_parfor_progress(N, i, t0);
            end

            fun_parfor_progress(N, -1, t0);
        else
            t0 = datetime('now');

            for i = 1:N
                [trajectory_samples(i).t, trajectory_samples(i).q, trajectory_samples(i).qd, trajectory_samples(i).qdd] = ...
                    fun_trajectory_constraint_actuator_limit(robot, trajectory_samples(i).t, trajectory_samples(i).q, ...
                    trajectory_samples(i).qd, trajectory_samples(i).qdd, transmission, joint_capability);
                fun_for_progress(N, i, t0);
            end

        end

    else
        error('constraint_mode is defined error!');
    end

    disp('Constrainted trajectory has been generated!')
    disp('----------------------------------------------------------------------------------------------------');

end
