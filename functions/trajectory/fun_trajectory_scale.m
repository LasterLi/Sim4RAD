function [t, q, qd, qdd] = fun_trajectory_scale(t, q, qd, qdd, trajectory_duration)

    T0 = t(end);
    dt = t(2) - t(1);

    time_scale = trajectory_duration / T0;

    t = t * time_scale;
    q = q;
    qd = qd / time_scale;
    qdd = qdd / time_scale ^ 2;

    t_new = 0:dt:trajectory_duration;

    dof = size(q, 2);

    q_new = zeros(length(t_new), dof);
    qd_new = zeros(length(t_new), dof);
    qdd_new = zeros(length(t_new), dof);

    for i = 1:dof
        q_new(:, i) = makima(t, q(:, i), t_new');
        qd_new(:, i) = makima(t, qd(:, i), t_new');
        qdd_new(:, i) = makima(t, qdd(:, i), t_new');
    end

    t = t_new;
    q = q_new;
    qd = qd_new;
    qdd = qdd_new;
end
