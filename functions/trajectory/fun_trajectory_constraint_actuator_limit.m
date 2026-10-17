function [t, q, qd, qdd] = fun_trajectory_constraint_actuator_limit(robot, t, q, qd, qdd, transmission, joint_capability)

    F = @(x) limit_err(x, robot, t, q, qd, qdd, transmission, joint_capability);
    options = optimoptions('fsolve', 'Display', 'off', 'MaxIterations', 50);
    trajectory_duration = fsolve(F, t(end), options);
    dt = t(2) - t(1);

    if trajectory_duration < 10 * dt
        trajectory_duration = 10 * dt;
    end

    [t, q, qd, qdd] = fun_trajectory_scale(t, q, qd, qdd, trajectory_duration);

end

function y = limit_err(x, robot, t, q, qd, qdd, transmission, joint_capability)
    dt = t(2) - t(1);

    if x < 10 * dt
        y = 1e6 * (10 * dt - x) ^ 0.5 + limit_err(10 * dt, robot, t, q, qd, qdd, transmission, joint_capability);
        return;
    end

    [t1, q1, qd1, qdd1] = fun_trajectory_scale(t, q, qd, qdd, x);
    [tau_motor, tau_out, power_motor, power_out] = fun_trajectory_inverse_dynamics(robot, t1, q1, qd1, qdd1, transmission, zeros(1, 6));
    dof = length(homeConfiguration(robot));
    theta = atan(joint_capability.motor_peak_torque ./ joint_capability.motor_peak_velocity);
    err = zeros(1, dof);

    for i = 1:dof
        R = [cos(theta(i)), -sin(theta(i));
             sin(theta(i)), cos(theta(i))];
        lim = joint_capability.motor_peak_torque(i) * cos(theta(i));

        if transmission.consider
            qd1_motor = qd1(:, i) * transmission.gear_ratio(i);
        else
            qd1_motor = qd1(:, i);
        end

        curve = abs([qd1_motor, tau_motor(:, i)]);
        curve_rotation = R * curve';
        err(i) = lim - max(curve_rotation(2, :));
    end

    y = min(err);

end
