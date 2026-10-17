function [t, q, qd, qdd, config] = fun_trajectory_from_datafile(robot, config)
    global error_threshold;

    dof = length(homeConfiguration(robot));

    data = csvread(config.dynamics.trajectory_data_file);
    [num_row, num_col] = size(data);

    config.dynamics.time_step = data(2, 1) - data(1, 1);
    disp(['The time step is changed to ', num2str(config.dynamics.time_step), '.']);

    if strcmpi(config.dynamics.planning_space, 'cartesian')
        disp('Provided trajectory data is in cartesian space. Using ik to solve the joint data.');

        if num_col == 1 + 7
            t = data(:, 1);
            pos = data(:, 2:8);

            q = zeros(num_row, dof);

            q_guess = zeros(1, dof);

            t0 = datetime('now');

            for i = 1:num_row
                [q(i, :), ~, flag] = fun_ik(robot, pos(i, :), [], q_guess, error_threshold);

                if flag == 0
                    error(['End pose at Line ', num2str(i), ' has no ik solution!']);
                end

                q_guess = q(i, :);

                fun_for_progress(num_row, i, t0);
            end

            [t, q, qd, qdd] = fun_trajectory_derivative(t, q);

        elseif num_col == 1 + 7 + 6
            t = data(:, 1);
            pos = data(:, 2:8);
            vel = data(:, 9:14);

            q = zeros(num_row, dof);
            qd = zeros(num_row, dof);

            q_guess = zeros(1, dof);
            t0 = datetime('now');

            for i = 1:num_row
                [q(i, :), qd(i, :), flag] = fun_ik(robot, pos(i, :), vel(i, :), q_guess, error_threshold);

                if flag == 0
                    error(['End pose at Line ', num2str(i), ' has no ik solution!']);
                end

                q_guess = q(i, :);

                fun_for_progress(num_row, i, t0);
            end

            [t, q, qd, qdd] = fun_trajectory_derivative(t, q, qd);

        else
            error(['Provided data in ', config.dynamics.trajectory_data_file, ' has wrong size!']);
        end

    elseif strcmpi(config.dynamics.planning_space, 'joint')
        disp('Provided trajectory data is in joint space.');

        if num_col == 1 + dof
            t = data(:, 1);
            q = data(:, 2:(dof + 1));
            [t, q, qd, qdd] = fun_trajectory_derivative(t, q);
        elseif num_col == 1 + 2 * dof
            t = data(:, 1);
            q = data(:, 2:(dof + 1));
            qd = data(:, (dof + 2):(2 * dof + 1));
            [t, q, qd, qdd] = fun_trajectory_derivative(t, q, qd);
        else
            error(['Provided data in ', config.dynamics.trajectory_data_file, ' has wrong size!']);
        end

    else
        error('Planning_space type is defined error! It should be cartesian or joint!');
    end

end
