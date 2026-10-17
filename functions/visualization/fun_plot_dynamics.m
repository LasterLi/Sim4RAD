function fun_plot_dynamics(robot, t, q, qd, qdd, ...
        pos_ee, vel_ee, vel_ee_norm, acc_ee, acc_ee_norm, tau, joint_peak_torque, joint_peak_velocity)

    dof = length(homeConfiguration(robot));

    disp([newline, 'Plot individual trajectory inverse dynamics simulation results:']);

    % plot end-effector position

    % plot joint state

    % plot end state
    figure(3)
    subplot(2, 3, 1);
    plot(t, vel_ee_norm, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('End total')
    set(gca, 'FontSize', 15);

    subplot(2, 3, 2);

    for i = 1:3
        plot(t, vel_ee(:, i), 'LineWidth', 2);
        hold on
    end

    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('linear x comp', 'linear y comp', 'linear z comp');
    set(gca, 'FontSize', 15);

    subplot(2, 3, 3);

    for i = 1:3
        plot(t, vel_ee(:, i + 3), 'LineWidth', 2);
        hold on
    end

    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend('angular x comp', 'angular y comp', 'angular z comp');
    set(gca, 'FontSize', 15);

    subplot(2, 3, 4);
    plot(t, acc_ee_norm, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend('End total')
    set(gca, 'FontSize', 15);

    subplot(2, 3, 5);

    for i = 1:3
        plot(t, acc_ee(:, i), 'LineWidth', 2);
        hold on
    end

    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend('linear x comp', 'linear y comp', 'linear z comp');
    set(gca, 'FontSize', 15);

    subplot(2, 3, 6);

    for i = 1:3
        plot(t, acc_ee(:, i + 3), 'LineWidth', 2);
        hold on
    end

    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    legend('angular x comp', 'angular y comp', 'angular z comp');
    set(gca, 'FontSize', 15);

    set(gcf, 'color', [1 1 1]);
    disp('Figure 3. End-effector velocity and acceleration vs time.');
    pause(0.1);

    % plot torque
    figure(4)
    num_panel = ceil(dof / 2);

    for i = 1:dof
        subplot(2, num_panel, i)
        plot(t, tau(:, i), 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Torque (Nm)');
        title(['Joint ', num2str(i)]);
        set(gca, 'FontSize', 15);
    end

    set(gcf, 'color', [1 1 1]);
    disp('Figure 4. Joint torque vs time.');
    pause(0.1);

    % plot power
    figure(5)
    num_panel = ceil(dof / 2);

    for i = 1:dof
        subplot(2, num_panel, i)
        plot(t, qd(:, i) .* tau(:, i), 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Power (W)');
        title(['Joint ', num2str(i)]);
        set(gca, 'FontSize', 15);
    end

    set(gcf, 'color', [1 1 1]);
    disp('Figure 5. Joint power vs time.');
    pause(0.1);

    % plot torque vs velcity
    figure(6)
    num_panel = ceil(dof / 2);

    for i = 1:dof
        subplot(2, num_panel, i)
        scatter(qd(:, i) * 60/2 / pi, tau(:, i));
        hold on

        if ~isempty(joint_peak_torque) && ~isempty(joint_peak_velocity)
            plot([0, joint_peak_velocity(i)], [joint_peak_torque(i), 0], 'k', 'LineWidth', 2)
            hold on
            plot([0, -joint_peak_velocity(i)], [-joint_peak_torque(i), 0], 'k', 'LineWidth', 2)
            hold on
        end

        xlabel('velocity (rpm)');
        ylabel('Torque (Nm)');
        title(['Joint ', num2str(i)]);
        set(gca, 'FontSize', 15);

    end

    set(gcf, 'color', [1 1 1]);
    disp('Figure 6. Joint torque vs velocity.');
    pause(0.1);
