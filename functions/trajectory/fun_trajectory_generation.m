function [t, q, qd, qdd, config] = fun_trajectory_generation(robot, config)
    % return the interpolation data of trajectory (t,q,qd,qdd)

    if config.dynamics.trajectory_type == 1
        disp(['Read the trajectory data from "', config.dynamics.trajectory_data_file, '".']);
        [t, q, qd, qdd, config] = fun_trajectory_from_datafile(robot, config);

    elseif config.dynamics.trajectory_type == 2
        disp('Interpolate the trajectory from the given initial state to end state.');

        if strcmpi(config.dynamics.planning_space, 'joint')

            q0 = config.dynamics.start_configuration;
            qd0 = config.dynamics.start_velocity;
            q1 = config.dynamics.end_configuration;
            qd1 = config.dynamics.end_velocity;
            dt = config.dynamics.time_step;
            interpolation_method = config.dynamics.interpolation_method;

            disp(['Initial joint configuration: ', num2str(q0)]);
            disp(['Initial joint velocity: ', num2str(qd0)]);
            disp(['End joint configuration: ', num2str(q1)]);
            disp(['End joint velocity: ', num2str(qd1)]);

            [t, q, qd, qdd] = fun_joint_interpolation(q0, qd0, q1, qd1, 1, dt, interpolation_method);

        elseif strcmpi(config.dynamics.planning_space, 'cartesian')
            pos0 = config.dynamics.start_configuration;
            vel0 = config.dynamics.start_velocity;
            pos1 = config.dynamics.end_configuration;
            vel1 = config.dynamics.end_velocity;
            dt = config.dynamics.time_step;
            interpolation_method = config.dynamics.interpolation_method;

            disp(['Initial end-effector position: ', num2str(pos0)]);
            disp(['Initial end-effector velocity: ', num2str(vel0)]);
            disp(['Final end-effector position: ', num2str(pos1)]);
            disp(['Final end-effector velocity: ', num2str(vel1)]);

            % The nominal duration of the interpolated motion is not the final
            % one: it is only used to build the primitive trajectory, and the
            % duration is set afterwards by fun_trajectory_constraint, which
            % rescales the whole trajectory. Any non-zero value works here.
            if strcmpi(config.dynamics.interpolation_space, 'joint')
                [t, q, qd, qdd] = fun_trajectory_interpolation_cartesian1(robot, pos0, vel0, pos1, vel1, 1, dt, interpolation_method);
            elseif strcmpi(config.dynamics.interpolation_space, 'cartesian')
                disp('Interpolate in the cartesian space, and then use ik to calculate joint qngles.');
                [t, q, qd, qdd] = fun_trajectory_interpolation_cartesian2(robot, pos0, vel0, pos1, vel1, 1, dt, interpolation_method);
            else
                error('interpolation_space is defined error!');
            end

        else
            error('planning_space is defined error!');
        end

    else
        error('trajectory_type is defined error!');
    end

    disp('----------------------------------------------------------------------------------------------------');

end
