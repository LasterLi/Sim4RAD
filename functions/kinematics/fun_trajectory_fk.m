function [pos, vel, acc, linear_vel, ang_vel, linear_acc, ang_acc] = fun_trajectory_fk(robot, t, q, qd, qdd)

    pos = zeros(length(t), 7);
    vel = zeros(length(t), 6);
    acc = zeros(length(t), 6);

    for i = 1:length(t)
        [pos(i, :), vel(i, :)] = fun_fk(robot, q(i, :), qd(i, :));
    end

    for i = 1:6
        acc(:, i) = fun_calc_first_derivative(t, vel(:, i));
    end

    linear_vel = zeros(length(t), 1);
    ang_vel = zeros(length(t), 1);
    linear_acc = zeros(length(t), 1);
    ang_acc = zeros(length(t), 1);

    for i = 1:length(t)
        linear_vel(i) = norm(vel(i, 1:3));
        ang_vel(i) = norm(vel(i, 4:6));
        linear_acc(i) = norm(acc(i, 1:3));
        ang_acc(i) = norm(acc(i, 4:6));
    end

end
