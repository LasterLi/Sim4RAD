function [t, q, qd, qdd] = fun_trajectory_interpolation_cartesian1(robot, pos0, vel0, pos1, vel1, T, dt, interpolation_method)

    global error_threshold;

    dof = length(homeConfiguration(robot));

    [q0, qd0, flag] = fun_ik(robot, pos0, vel0, zeros(1, dof), error_threshold);

    if flag == 0
        error(['start_configuration has no ik solution!']);
    end

    [q1, qd1, flag] = fun_ik(robot, pos1, vel1, q0, error_threshold);

    if flag == 0
        error(['end_configuration has no ik solution!']);
    end

    [t, q, qd, qdd] = fun_joint_interpolation(q0, qd0, q1, qd1, T, dt, interpolation_method);
end
