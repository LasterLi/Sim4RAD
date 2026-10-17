function fun_plot_joint_trajectory(t, q, qd, qdd, if_together)

    dof = size(q, 2);

    if if_together
        figure;

        legend_array = strings(1, dof);

        for i = 1:dof
            legend_array(i) = ['Joint ', num2str(i)];
        end

        subplot(1, 3, 1)
        plot(t, q, 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Joint position (rad)');
        legend(legend_array)
        set(gca, 'FontSize', 15);

        subplot(1, 3, 2)
        plot(t, qd, 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Joint velocity (rad/s)');
        set(gca, 'FontSize', 15);

        subplot(1, 3, 3)
        plot(t, qdd, 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Joint acceleration (rad/s^2)');
        set(gca, 'FontSize', 15);

    else
        figure;

        for i = 1:dof
            subplot(3, dof, i)
            plot(t, q(:, i), 'LineWidth', 2);
            xlabel('Time (s)');
            ylabel('Position (rad)');
            title(['Joint ', num2str(i)]);
            set(gca, 'FontSize', 15);

            subplot(3, dof, dof + i)
            plot(t, qd(:, i), 'LineWidth', 2);
            xlabel('Time (s)');
            ylabel('Velocity (rad/s)');
            title(['Joint ', num2str(i)]);
            set(gca, 'FontSize', 15);

            subplot(3, dof, 2 * dof + i)
            plot(t, qdd(:, i), 'LineWidth', 2);
            xlabel('Time (s)');
            ylabel('Acceleration (rad/s^2)');
            title(['Joint ', num2str(i)]);
            set(gca, 'FontSize', 15);
        end

    end

    set(gcf, 'color', [1 1 1]);
    disp('Figure: Joint state vs time.');
    pause(0.1);
end
