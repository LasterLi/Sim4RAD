function [t, q, qd, qdd] = fun_trajectory_interpolation_cartesian2(robot, pos0, vel0, pos1, vel1, T, dt, interpolation_method)

    global error_threshold;

    dof = length(homeConfiguration(robot));

    [t, pos, vel] = fun_pose_interpolation(pos0, vel0, pos1, vel1, T, dt, interpolation_method);

    q = zeros(length(t), dof);
    qd = zeros(length(t), dof);

    q_guess = zeros(1, dof);
    t0 = datetime('now');

    for i = 1:length(t)
        [q(i, :), qd(i, :), flag] = fun_ik(robot, pos(i, :), vel(i, :), q_guess, error_threshold);

        if flag == 0
            error(['End pose at step ', num2str(i), ' has no ik solution!']);
        end

        q_guess = q(i, :);
        fun_for_progress(length(t), i, t0);
    end

    [t, q, qd, qdd] = fun_trajectory_derivative(t, q, qd);
end
