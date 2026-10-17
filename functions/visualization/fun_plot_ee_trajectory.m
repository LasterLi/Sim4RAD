function fun_plot_ee_trajectory(t, pos, vel, acc, linear_vel, ang_vel, linear_acc, ang_acc)

    % ee position, ee velocity, ee acceleration
    figure;
    subplot(2, 3, 1)
    plot(t, pos(:, 1:3), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('EE position (m)');
    legend('x', 'y', 'z')
    set(gca, 'FontSize', 15);

    subplot(2, 3, 2)
    plot(t, vel(:, 1:3), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('EE velocity (m/s)');
    set(gca, 'FontSize', 15);

    subplot(2, 3, 3)
    plot(t, acc(:, 1:3), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('EE acceleration (m/s^2)');
    set(gca, 'FontSize', 15);

    subplot(2, 3, 5)
    plot(t, linear_vel, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('EE velocity (m/s)');
    set(gca, 'FontSize', 15);

    subplot(2, 3, 6)
    plot(t, linear_acc, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('EE acceleration (m/s^2)');
    set(gca, 'FontSize', 15);

    set(gcf, 'color', [1 1 1]);
    disp('Figure: End-effector position state vs time.');
    pause(0.1);

    % ee angular velocity, ee angular acceleration (x,y,z)
    figure;

    subplot(2, 2, 1)
    plot(t, vel(:, 4:6), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('EE angular velocity (rad/s)');
    legend('x', 'y', 'z')
    set(gca, 'FontSize', 15);

    subplot(2, 2, 2)
    plot(t, acc(:, 4:6), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('EE angular acceleration (rad/s^2)');
    set(gca, 'FontSize', 15);

    subplot(2, 2, 3)
    plot(t, ang_vel, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('EE angular velocity (rad/s)');
    set(gca, 'FontSize', 15);

    subplot(2, 2, 4)
    plot(t, ang_acc, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('EE angular acceleration (rad/s^2)');
    set(gca, 'FontSize', 15);

    set(gcf, 'color', [1 1 1]);
    disp('Figure: End-effector orientation state vs time.');
    pause(0.1);

end
