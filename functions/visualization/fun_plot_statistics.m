function fun_plot_statistics(statistics, joint_capability)

    % statistics data in bar chart
    figure;

    subplot(1, 4, 1)
    histogram(statistics.duration, 10);
    xlabel('duration (s)');
    set(gca, 'FontSize', 15);

    subplot(1, 4, 2)
    histogram(statistics.dis, 10);
    xlabel('dis (m)');
    set(gca, 'FontSize', 15);

    subplot(1, 4, 3)
    histogram(statistics.max_ee_vel, 10);
    xlabel('max\_ee\_vel (m/s)');
    set(gca, 'FontSize', 15);

    subplot(1, 4, 4)
    histogram(statistics.max_ee_acc, 10);
    xlabel('max\_ee\_acc (m/s^2)');
    set(gca, 'FontSize', 15);

    set(gcf, 'color', [1 1 1]);
    disp('Figure: kinematic statistics data.');
    pause(0.1);

    figure;

    dof = size(statistics.max_joint_torque, 2);
    num = length(statistics.max_joint_torque(:, 1));

    for i = 1:dof
        subplot(4, dof, i)
        histogram(statistics.max_joint_torque(:, i), 10);
        hold on
        plot(ones(1, 2) * joint_capability.joint_peak_torque(i), [0, num / 2], 'k--', 'Linewidth', 2);
        xlabel('max\_joint\_torque (Nm)');
        title(['Joint ', num2str(i)]);
        set(gca, 'FontSize', 15);

        subplot(4, dof, dof + i)
        histogram(statistics.max_joint_vel(:, i) * 60/2 / pi, 10);
        hold on
        plot(ones(1, 2) * joint_capability.joint_peak_velocity(i) * 60/2 / pi, [0, num / 2], 'k--', 'Linewidth', 2);
        xlabel('max\_joint\_vel (rpm)');
        set(gca, 'FontSize', 15);

        subplot(4, dof, 2 * dof + i)
        histogram(statistics.max_motor_torque(:, i), 10);
        hold on
        plot(ones(1, 2) * joint_capability.motor_peak_torque(i), [0, num / 2], 'k--', 'Linewidth', 2);
        xlabel('max\_motor\_torque (Nm)');
        set(gca, 'FontSize', 15);

        subplot(4, dof, 3 * dof + i)
        histogram(statistics.max_motor_vel(:, i) * 60/2 / pi, 10);
        hold on
        plot(ones(1, 2) * joint_capability.motor_peak_velocity(i) * 60/2 / pi, [0, num / 2], 'k--', 'Linewidth', 2);
        xlabel('max\_motor\_vel (rpm)');
        set(gca, 'FontSize', 15);
    end

    set(gcf, 'color', [1 1 1]);
    disp('Figure: dynamics statistics data.');
    pause(0.1);

    % ee_vel, ee_acc, ee_dis
    figure;
    subplot(1, 2, 1)
    scatter(statistics.dis, statistics.max_ee_vel);
    hold on
    [mean_x, mean_y, std_x, std_y] = fun_scatter_data_statistics(statistics.dis, statistics.max_ee_vel);
    errorbar(mean_x, mean_y, std_y, std_y, std_x, std_x, 'r', 'LineWidth', 3)
    xlabel('distance (m)');
    ylabel('max\_ee\_vel (m/s)');
    set(gca, 'FontSize', 15);

    subplot(1, 2, 2)
    scatter(statistics.dis, statistics.max_ee_acc);
    hold on
    [mean_x, mean_y, std_x, std_y] = fun_scatter_data_statistics(statistics.dis, statistics.max_ee_acc);
    errorbar(mean_x, mean_y, std_y, std_y, std_x, std_x, 'r', 'LineWidth', 3)
    xlabel('distance (m)');
    ylabel('max\_ee\_acc (m/s^2)');
    set(gca, 'FontSize', 15);

    set(gcf, 'color', [1 1 1]);
    disp('Figure: distance vs max_ee_vel, and vs max_ee_acc.');
    pause(0.1);

end
