function [t, q, qd, qdd] = fun_trajectory_constraint(robot, t, q, qd, qdd, config)

    if config.dynamics.constraint_mode == 1
        duration = config.dynamics.trajectory_duration;
        disp(['trajectory_constraint is duration = ', num2str(duration), ' s']);
        [t, q, qd, qdd] = fun_trajectory_scale(t, q, qd, qdd, duration);
    elseif config.dynamics.constraint_mode == 2
        max_ee_velocity = config.dynamics.max_ee_velocity;
        disp(['trajectory_constraint is max_ee_velocity = ', num2str(max_ee_velocity), ' m/s']);
        [t, q, qd, qdd] = fun_trajectory_constraint_max_ee_velocity(robot, t, q, qd, qdd, max_ee_velocity);
    elseif config.dynamics.constraint_mode == 3
        max_ee_acceleration = config.dynamics.max_ee_acceleration;
        disp(['trajectory_constraint is max_ee_acceleration = ', num2str(max_ee_acceleration), ' m/s^2']);
        [t, q, qd, qdd] = fun_trajectory_constraint_max_ee_acceleration(robot, t, q, qd, qdd, max_ee_acceleration);
    elseif config.dynamics.constraint_mode == 4
        [transmission, joint_capability] = fun_get_joint_dynamics_capability(config.dynamics);
        disp('trajectory_constraint is joint_capability.');
        disp(['Maximal joint torque is: ', num2str(config.dynamics.joint_peak_torque)]);
        disp(['Maximal joint velocity is: ', num2str(config.dynamics.joint_peak_velocity)]);
        [t, q, qd, qdd] = fun_trajectory_constraint_actuator_limit(robot, t, q, qd, qdd, transmission, joint_capability);
    else
        error('constraint_mode is defined error!');
    end

    disp('Constrainted trajectory has been generated!')
    disp('----------------------------------------------------------------------------------------------------');

end
