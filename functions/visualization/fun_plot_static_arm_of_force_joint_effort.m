function fun_plot_static_arm_of_force_joint_effort(arm_of_force, tau, expected_arm_of_force)

    dof = size(tau, 2);
    num_col = ceil(dof / 2);

    max_torque = zeros(1, dof);
    max_torque_less_than_expected = zeros(1, dof);
    figure;

    for i = 1:dof
        subplot(2, num_col, i)
        scatter(arm_of_force, tau(:, i));
        hold on
        plot(expected_arm_of_force * ones(1, 2), [0, 1.1 * max(tau(:, i))], 'k--', 'LineWidth', 2)
        xlabel('arm of force (m)');
        ylabel('joint torque (Nm)');
        title(['Joint ', num2str(i)]);
        set(gca, 'FontSize', 15);

        max_torque(i) = max(tau(:, i));
        max_torque_less_than_expected(i) = max(tau(arm_of_force <= expected_arm_of_force, i));
    end

    disp(['The max joint torques are: ', num2str(max_torque)]);
    disp(['When arm of force is less than ', num2str(expected_arm_of_force), ' m, the max joint torques are: ', num2str(max_torque_less_than_expected)]);

    set(gcf, 'color', [1 1 1]);
    pause(0.1);
