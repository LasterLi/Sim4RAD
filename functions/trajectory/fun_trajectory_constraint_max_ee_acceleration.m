function [t, q, qd, qdd] = fun_trajectory_constraint_max_ee_acceleration(robot, t, q, qd, qdd, max_ee_acceleration)

    F = @(x) max_ee_acceleration_err(x, robot, t, q, qd, qdd, max_ee_acceleration);
    options = optimoptions('fsolve', 'Display', 'off');
    trajectory_duration = fsolve(F, t(end), options);
    [t, q, qd, qdd] = fun_trajectory_scale(t, q, qd, qdd, trajectory_duration);

end

function y = max_ee_acceleration_err(x, robot, t, q, qd, qdd, max_ee_acceleration)
    dt = t(2) - t(1);

    if x < 10 * dt
        y = 100 * (10 * dt - x) + max_ee_acceleration_err(10 * dt, robot, t, q, qd, qdd, max_ee_acceleration);
        return;
    end

    [t1, q1, qd1, qdd1] = fun_trajectory_scale(t, q, qd, qdd, x);
    [pos, vel, acc, linear_vel, ang_vel, linear_acc, ang_acc] = fun_trajectory_fk(robot, t1, q1, qd1, qdd1);
    y = (max(linear_acc) - max_ee_acceleration);

end
