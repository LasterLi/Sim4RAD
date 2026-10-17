function [t, q, qd, qdd] = fun_joint_interpolation(q0, qd0, q1, qd1, T, dt, interpolation_method)

    dof = length(q0);

    q0 = q0(:)';
    q1 = q1(:)';
    qd0 = qd0(:)';
    qd1 = qd1(:)';

    t = 0:dt:T;

    q = zeros(length(t), dof);
    qd = zeros(length(t), dof);
    qdd = zeros(length(t), dof);

    if strcmpi(interpolation_method, 'cubic')

        for i = 1:dof
            q(:, i) = fun_poly3(t, 0, q0(i), qd0(i), T, q1(i), qd1(i), 0);
            qd(:, i) = fun_poly3(t, 0, q0(i), qd0(i), T, q1(i), qd1(i), 1);
            qdd(:, i) = fun_poly3(t, 0, q0(i), qd0(i), T, q1(i), qd1(i), 2);
        end

    elseif strcmpi(interpolation_method, 'quintic')

        for i = 1:dof
            q(:, i) = fun_poly5(t, 0, q0(i), qd0(i), 0, T, q1(i), qd1(i), 0, 0);
            qd(:, i) = fun_poly5(t, 0, q0(i), qd0(i), 0, T, q1(i), qd1(i), 0, 1);
            qdd(:, i) = fun_poly5(t, 0, q0(i), qd0(i), 0, T, q1(i), qd1(i), 0, 2);
        end

    else
        error('Interpolation method is defined error!');
    end
