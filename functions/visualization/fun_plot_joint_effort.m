function fun_plot_joint_effort(t, tau_motor, tau_out, power_motor, power_out, joint_capability, if_together)
    dof = size(tau_motor, 2);

    if if_together
        figure;

        legend_array = strings(1, dof);

        for i = 1:dof
            legend_array(i) = ['Joint ', num2str(i)];
        end

        subplot(2, 2, 1)
        plot(t, tau_motor, 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('tua motor (Nm)');
        legend(legend_array)
        set(gca, 'FontSize', 15);

        subplot(2, 2, 2)
        plot(t, power_motor, 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('power motor (W)');
        set(gca, 'FontSize', 15);

        subplot(2, 2, 3)
        plot(t, tau_out, 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('tau out (Nm)');
        set(gca, 'FontSize', 15);

        subplot(2, 2, 4)
        plot(t, power_out, 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('power out (W)');
        set(gca, 'FontSize', 15);

    else
        figure;

        for i = 1:dof
            subplot(4, dof, i)
            plot(t, tau_motor(:, i), 'LineWidth', 2);
            hold on
            % plot([t(1), t(end)], ones(1, 2) * joint_capability.motor_peak_torque(i), 'k--', 'LineWidth', 2)
            xlabel('Time (s)');
            ylabel('tua motor (Nm)');
            title(['Joint ', num2str(i)]);
            set(gca, 'FontSize', 15);

            subplot(4, dof, dof + i)
            plot(t, tau_out(:, i), 'LineWidth', 2);
            hold on
            % plot([t(1), t(end)], ones(1, 2) * joint_capability.joint_peak_torque(i), 'k--', 'LineWidth', 2)
            xlabel('Time (s)');
            ylabel('tau out (Nm)');
            title(['Joint ', num2str(i)]);
            set(gca, 'FontSize', 15);

            subplot(4, dof, 2 * dof + i)
            plot(t, power_motor(:, i), 'LineWidth', 2);
            xlabel('Time (s)');
            ylabel('power motor (W)');
            title(['Joint ', num2str(i)]);
            set(gca, 'FontSize', 15);

            subplot(4, dof, 3 * dof + i)
            plot(t, power_out(:, i), 'LineWidth', 2);
            xlabel('Time (s)');
            ylabel('power out (W)');
            title(['Joint ', num2str(i)]);
            set(gca, 'FontSize', 15);
        end

    end

    set(gcf, 'color', [1 1 1]);
    disp('Figure: Joint effort vs time.');
    pause(0.1);
end
