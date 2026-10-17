function fun_plot_statistics_joint_effort(statistics, joint_capability, expected_dis)

    dof = size(statistics.max_joint_torque, 2);
    joint_requirements = zeros(1, dof);
    num_col = ceil(dof / 2);

    figure;

    for i = 1:dof
        subplot(2, num_col, i)
        scatter(statistics.dis, statistics.max_joint_torque(:, i));
        hold on
        [mean_x, mean_y, std_x, std_y] = fun_scatter_data_statistics(statistics.dis, statistics.max_joint_torque(:, i));
        errorbar(mean_x, mean_y, std_y, std_y, std_x, std_x, 'r', 'LineWidth', 3)
        hold on
        plot([min(statistics.dis), max(statistics.dis)], ones(1, 2) * joint_capability.joint_peak_torque(i), 'k--', 'Linewidth', 2);
        xlabel('distance (m)');
        ylabel('max\_joint\_torque (Nm)');
        title(['Joint ', num2str(i)]);
        ylim([0, max(mean_y) + max(std_y) * 2])
        set(gca, 'FontSize', 15);

        tmp = sort(statistics.max_joint_torque(statistics.dis >= expected_dis, i));
        joint_requirements(i) = tmp(round(length(tmp) * 0.9772));
    end

    disp(['To ensure the expected dynamical property less the motion distance ', num2str(expected_dis), ' m, the max joint torques are (Nm): ', num2str(joint_requirements)]);
    set(gcf, 'color', [1 1 1]);
    pause(0.1);

    figure;

    for i = 1:dof
        subplot(2, num_col, i)
        scatter(statistics.dis, statistics.max_joint_vel(:, i) * 60/2 / pi);
        hold on
        [mean_x, mean_y, std_x, std_y] = fun_scatter_data_statistics(statistics.dis, statistics.max_joint_vel(:, i) * 60/2 / pi);
        errorbar(mean_x, mean_y, std_y, std_y, std_x, std_x, 'r', 'LineWidth', 3)
        hold on
        plot([min(statistics.dis), max(statistics.dis)], ones(1, 2) * joint_capability.joint_peak_velocity(i) * 60/2 / pi, 'k--', 'Linewidth', 2);
        xlabel('distance (m)');
        ylabel('max\_joint\_vel (rpm)');
        title(['Joint ', num2str(i)]);
        ylim([0, max(mean_y) + max(std_y) * 2])
        set(gca, 'FontSize', 15);

        tmp = sort(statistics.max_joint_vel(statistics.dis >= expected_dis, i));
        joint_requirements(i) = tmp(round(length(tmp) * 0.9772)) * 60/2 / pi;
    end

    disp(['To ensure the expected dynamical property less the motion distance ', num2str(expected_dis), ' m, the max joint velocities are (rpm): ', num2str(joint_requirements)]);
    set(gcf, 'color', [1 1 1]);
    pause(0.1);

    figure;

    for i = 1:dof

        subplot(2, num_col, i)
        scatter(statistics.dis, statistics.max_motor_torque(:, i));
        hold on
        [mean_x, mean_y, std_x, std_y] = fun_scatter_data_statistics(statistics.dis, statistics.max_motor_torque(:, i));
        errorbar(mean_x, mean_y, std_y, std_y, std_x, std_x, 'r', 'LineWidth', 3)
        hold on
        plot([min(statistics.dis), max(statistics.dis)], ones(1, 2) * joint_capability.motor_peak_torque(i), 'k--', 'Linewidth', 2);
        xlabel('distance (m)');
        ylabel('max\_motor\_torque (Nm)');
        title(['joint ', num2str(i)]);
        ylim([0, max(mean_y) + max(std_y) * 2])
        set(gca, 'FontSize', 15);

        tmp = sort(statistics.max_motor_torque(statistics.dis >= expected_dis, i));
        joint_requirements(i) = tmp(round(length(tmp) * 0.9772));
    end

    disp(['To ensure the expected dynamical property less the motion distance ', num2str(expected_dis), ' m, the max motor torques are: ', num2str(joint_requirements)]);
    set(gcf, 'color', [1 1 1]);
    pause(0.1);

    figure;

    for i = 1:dof

        subplot(2, num_col, i)
        scatter(statistics.dis, statistics.max_motor_vel(:, i) * 60/2 / pi);
        hold on
        [mean_x, mean_y, std_x, std_y] = fun_scatter_data_statistics(statistics.dis, statistics.max_motor_vel(:, i) * 60/2 / pi);
        errorbar(mean_x, mean_y, std_y, std_y, std_x, std_x, 'r', 'LineWidth', 3)
        hold on
        plot([min(statistics.dis), max(statistics.dis)], ones(1, 2) * joint_capability.motor_peak_velocity(i) * 60/2 / pi, 'k--', 'Linewidth', 2);
        xlabel('distance (m)');
        ylabel('max\_motor\_vel (rpm)');
        title(['joint ', num2str(i)]);
        ylim([0, max(mean_y) + max(std_y) * 2])
        set(gca, 'FontSize', 15);

        tmp = sort(statistics.max_motor_vel(statistics.dis >= expected_dis, i));
        joint_requirements(i) = tmp(round(length(tmp) * 0.9772)) * 60/2 / pi;
    end

    disp(['To ensure the expected dynamical property less the motion distance ', num2str(expected_dis), ' m, the max motor velocities are(rpm): ', num2str(joint_requirements)]);
    set(gcf, 'color', [1 1 1]);
    disp('Figure: dynamics statistics data.');
    pause(0.1);

end
