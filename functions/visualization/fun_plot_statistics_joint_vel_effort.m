function fun_plot_statistics_joint_vel_effort(trajectory_samples, joint_capability, transmission)

    dof = size(trajectory_samples(1).q, 2);
    num_col = ceil(dof / 2);
    num = length(trajectory_samples);

    figure;

    for i = 1:dof
        subplot(2, num_col, i)
        t0 = datetime('now');

        for j = 1:num
            plot(abs(trajectory_samples(j).qd(:, i)) * 60/2 / pi, abs(trajectory_samples(j).tau_out(:, i)), 'g');
            hold on
            fun_for_progress(num, j, t0);
        end

        plot([0, joint_capability.joint_peak_velocity(i)] * 60/2 / pi, [joint_capability.joint_peak_torque(i), 0], 'k--', 'LineWidth', 2);
        xlabel('joint velocity (rpm)');
        ylabel('joint torque (Nm)');
        title(['Joint ', num2str(i)]);
        set(gca, 'FontSize', 15);
    end

    set(gcf, 'color', [1 1 1]);
    pause(0.1);

    figure;

    for i = 1:dof
        subplot(2, num_col, i)
        t0 = datetime('now');

        for j = 1:num
            plot(abs(trajectory_samples(j).qd(:, i)) * transmission.gear_ratio(i) * 60/2 / pi, abs(trajectory_samples(j).tau_motor(:, i)), 'g');
            hold on
            fun_for_progress(num, j, t0);
        end

        plot([0, joint_capability.motor_peak_velocity(i)] * 60/2 / pi, [joint_capability.motor_peak_torque(i), 0], 'k--', 'LineWidth', 2);
        xlabel('motor velocity (rpm)');
        ylabel('motor torque (Nm)');
        title(['Joint ', num2str(i)]);
        set(gca, 'FontSize', 15);
    end

    set(gcf, 'color', [1 1 1]);

    % set(gcf, 'color', [1 1 1]);
    disp('Figure: dynamics statistics data.');
    pause(0.1);

end
