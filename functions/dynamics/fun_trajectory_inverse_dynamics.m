function [tau_motor, tau_out, power_motor, power_out] = fun_trajectory_inverse_dynamics(robot, t, q, qd, qdd, transmission, end_wrench)

    dof = length(homeConfiguration(robot));
    LastLink = robot.BodyNames{1, end};
    tau_out = zeros(length(t), dof);

    for i = 1:length(t)

        T = getTransform(robot, q(i, :), LastLink);
        r = T(1:3, 4)';
        F = end_wrench(4:6);
        T = cross(r, F) + end_wrench(1:3);

        fext = externalForce(robot, LastLink, [T, F]);
        tau_out(i, :) = inverseDynamics(robot, q(i, :), qd(i, :), qdd(i, :), fext);
    end

    qd_motor = qd;
    tau_motor = tau_out;

    if transmission.consider

        for i = 1:dof
            tau_motor(:, i) = transmission.motor_inertia(i) * qdd(:, i) * transmission.gear_ratio(i) + ...
                + tau_out(:, i) / transmission.gear_ratio(i) / transmission.efficiency(i);

            qd_motor(:, i) = qd(:, i) * transmission.gear_ratio(i);

        end

    end

    power_motor = tau_motor .* qd_motor;
    power_out = tau_out .* qd;

end
