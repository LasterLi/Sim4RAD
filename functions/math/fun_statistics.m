function statistics = fun_statistics(trajectory_samples, transmission)
    statistics = struct;
    N = length(trajectory_samples);
    dof = size(trajectory_samples(1).q, 2);

    statistics.dis = zeros(N, 1);
    statistics.max_ee_vel = zeros(N, 1);
    statistics.max_ee_acc = zeros(N, 1);
    statistics.max_joint_torque = zeros(N, dof);
    statistics.max_joint_vel = zeros(N, dof);
    statistics.max_motor_torque = zeros(N, dof);
    statistics.max_motor_vel = zeros(N, dof);

    for i = 1:N
        statistics.duration(i) = trajectory_samples(i).t(end);
        statistics.dis(i) = trajectory_samples(i).dis;
        statistics.max_ee_vel(i) = max(trajectory_samples(i).linear_vel);
        statistics.max_ee_acc(i) = max(trajectory_samples(i).linear_acc);
        statistics.max_joint_torque(i, :) = max(abs(trajectory_samples(i).tau_out));
        statistics.max_joint_vel(i, :) = max(abs(trajectory_samples(i).qd));
        statistics.max_motor_torque(i, :) = max(abs(trajectory_samples(i).tau_motor));

        if transmission.consider
            statistics.max_motor_vel(i, :) = max(abs(trajectory_samples(i).qd)) .* transmission.gear_ratio;
        else
            statistics.max_motor_vel(i, :) = max(abs(trajectory_samples(i).qd));
        end

    end

end
