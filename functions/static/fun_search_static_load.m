function [tau_max, q_all, tau_all] = fun_search_static_load(robot, config)

    dof = length(homeConfiguration(robot));
    q_groups = cell(1, dof);

    for i = 1:dof
        q_groups_i = [config.modeling.joint_lower_limit(i), fliplr(0:-pi / 2:config.modeling.joint_lower_limit(i)), pi / 2:pi / 2:config.modeling.joint_upper_limit(i), config.modeling.joint_upper_limit(i)];

        if abs(q_groups_i(1) - q_groups_i(2)) < 5 * pi / 180
            q_groups_i(1) = [];
        end

        if abs(q_groups_i(end) - q_groups_i( end - 1)) < (5 * pi / 180)
            q_groups_i(end) = [];
        end

        q_groups{i} = q_groups_i;
    end

    q_all = fun_traversal(q_groups);
    num_all = size(q_all, 1);

    % calculate joint torques
    tau_all = zeros(num_all, dof);

    t0 = datetime('now');
    for i = 1:num_all
        tau_all(i, :) = abs(gravityTorque(robot, q_all(i, :)));
        fun_for_progress(num_all, i, t0);
    end

    tau_max = max(tau_all);
end
