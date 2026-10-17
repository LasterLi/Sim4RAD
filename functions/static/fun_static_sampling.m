function [q, arm_of_force, tau] = fun_static_sampling(robot, joint_lower_limits, joint_upper_limits, num_samples)

    dof = length(homeConfiguration(robot));
    q = rand(num_samples, dof) .* (joint_upper_limits - joint_lower_limits) + joint_lower_limits;
    LastLink = robot.BodyNames{1, end};

    arm_of_force = zeros(num_samples, 1);
    tau = zeros(num_samples, dof);

    t0 = datetime('now');

    for i = 1:num_samples
        T = getTransform(robot, q(i, :), LastLink);
        arm_of_force(i) = norm(T(1:2, 4));
        tau(i, :) = abs(gravityTorque(robot, q(i, :)));
        fun_for_progress(num_samples, i, t0);
    end

end
