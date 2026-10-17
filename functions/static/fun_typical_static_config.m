function [q_typical, tau_typical, max_joint_id] = fun_typical_static_config(q_all, tau_all)
    % Merge similar results:
    % Poses that have the same joint torques only leave the one closest to home configuration.
    dof = size(q_all, 2);
    num_all = size(q_all, 1);
    qq_data = q_all(1, :);
    tau_data = tau_all(1, :);

    t0 = datetime('now');

    for i = 2:num_all
        kk = 0;

        for k = 1:size(qq_data, 1)

            if norm(tau_all(i, :) - tau_data(k, :)) < 1e-3
                kk = k;

                if norm(qq_data(k, :)) > norm(q_all(i, :))
                    qq_data(k, :) = q_all(i, :);
                    tau_data(k, :) = tau_all(i, :);
                end

                break
            end

        end

        if kk == 0
            qq_data = [qq_data; q_all(i, :)];
            tau_data = [tau_data; tau_all(i, :)];
        end

        fun_for_progress(num_all - 1, i - 1, t0);
    end

    % Find typical poses
    joint_max_load = max(tau_all);
    typical_poses = zeros(dof, dof);
    typical_poses_tau = zeros(dof, dof);
    typical_poses_max_id = zeros(dof, dof);

    for i = 1:dof
        % Find the poses that the i^th joint has the maximal torque.
        imax_tau = tau_data(abs(tau_data(:, i) - joint_max_load(i)) < 1e-3, :);
        imax_qq = qq_data(abs(tau_data(:, i) - joint_max_load(i)) < 1e-3, :);

        imax_tau_norm = zeros(1, size(imax_tau, 1));

        for j = 1:size(imax_tau, 1)
            imax_tau_norm(j) = norm(imax_tau(j, :));
        end

        % Keep the one of all joint torques are large as candidate.
        [~, max_idx] = max(imax_tau_norm);
        typical_poses(i, :) = imax_qq(max_idx, :);
        typical_poses_tau(i, :) = imax_tau(max_idx, :);
        typical_poses_max_id(i, :) = (abs(typical_poses_tau(i, :) - joint_max_load) < 1e-3);
    end

    % Merge similar results.
    [typical_poses, leave_idx] = fun_merge_data(typical_poses, 1, 1e-3);
    typical_poses_tau = typical_poses_tau(leave_idx, :);
    typical_poses_max_id = typical_poses_max_id(leave_idx, :);

    % Find a minimal combination.
    leave_idx = [];
    have_dof = zeros(1, dof);
    sum_dof = sum(have_dof);

    for i = 1:size(typical_poses, 1)

        for j = 1:dof

            if typical_poses_max_id(i, j)
                have_dof(j) = 1;
            end

        end

        if sum(have_dof) > sum_dof
            leave_idx = [leave_idx, i];
        end

        sum_dof = sum(have_dof);

        if sum_dof == dof
            break;
        end

    end

    q_typical = typical_poses(leave_idx, :);
    tau_typical = typical_poses_tau(leave_idx, :);
    max_joint_id = typical_poses_max_id(leave_idx, :);
end
