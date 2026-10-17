function fun_plot_joint_vel_effort(t, tau_motor, tau_out, qd, transmission, joint_capability)
    figure;
    dof = size(tau_motor, 2);

    if transmission.consider
        qd_motor = qd .* transmission.gear_ratio;
    else
        qd_motor = qd;
    end

    for i = 1:dof
        subplot(2, dof, i)
        plot(abs(qd_motor(:, i)) * 60/2 / pi, abs(tau_motor(:, i)), 'LineWidth', 2);
        hold on
        plot([0, joint_capability.motor_peak_velocity(i)] * 60/2 / pi, [joint_capability.motor_peak_torque(i), 0], 'k--', 'LineWidth', 2);
        xlabel('motor velocity (rpm)');
        ylabel('motor torque (Nm)');
        title(['Joint ', num2str(i)]);
        set(gca, 'FontSize', 15);

        subplot(2, dof, dof + i)
        plot(abs(qd(:, i)) * 60/2 / pi, abs(tau_out(:, i)), 'LineWidth', 2);
        hold on
        plot([0, joint_capability.joint_peak_velocity(i)] * 60/2 / pi, [joint_capability.joint_peak_torque(i), 0], 'k--', 'LineWidth', 2);
        xlabel('joint velocity (rpm)');
        ylabel('joint torque (Nm)');
        title(['Joint ', num2str(i)]);
        set(gca, 'FontSize', 15);

    end

    set(gcf, 'color', [1 1 1]);
    disp('Figure: Joint effort vs joint velocity.');
    pause(0.1);
end
