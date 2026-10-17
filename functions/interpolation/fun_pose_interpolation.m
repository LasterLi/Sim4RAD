function [t, pos, vel] = fun_pose_interpolation(pos0, vel0, pos1, vel1, T, dt, interpolation_method)

    x0 = pos0(1);
    y0 = pos0(2);
    z0 = pos0(3);

    xd0 = vel0(1);
    yd0 = vel0(2);
    zd0 = vel0(3);

    x1 = pos1(1);
    y1 = pos1(2);
    z1 = pos1(3);

    xd1 = vel1(1);
    yd1 = vel1(2);
    zd1 = vel1(3);

    t = 0:dt:T;

    if strcmpi(interpolation_method, 'cubic')

        x = fun_poly3(t, 0, x0, xd0, T, x1, xd1, 0);
        xd = fun_poly3(t, 0, x0, xd0, T, x1, xd1, 1);
        y = fun_poly3(t, 0, y0, yd0, T, y1, yd1, 0);
        yd = fun_poly3(t, 0, y0, yd0, T, y1, yd1, 1);
        z = fun_poly3(t, 0, z0, zd0, T, z1, zd1, 0);
        zd = fun_poly3(t, 0, z0, zd0, T, z1, zd1, 1);

    elseif strcmpi(interpolation_method, 'quintic')

        x = fun_poly5(t, 0, x0, xd0, 0, T, x1, xd1, 0, 0);
        xd = fun_poly5(t, 0, x0, xd0, 0, T, x1, xd1, 0, 1);
        y = fun_poly5(t, 0, y0, yd0, 0, T, y1, yd1, 0, 0);
        yd = fun_poly5(t, 0, y0, yd0, 0, T, y1, yd1, 0, 1);
        z = fun_poly5(t, 0, z0, zd0, 0, T, z1, zd1, 0, 0);
        zd = fun_poly5(t, 0, z0, zd0, 0, T, z1, zd1, 0, 1);

    else
        error('Interpolation method is defined error!');
    end

    % rotation motion use eular angle interpolation
    quat0 = pos0(4:7);
    ang_vel_0 = vel0(4:6);
    quat1 = pos1(4:7);
    ang_vel_1 = vel1(4:6);

    eul0 = quat2eul(quat0);
    eul1 = quat2eul(quat1);
    eul_vel0 = fun_angle_velocity_to_eular(ang_vel_0, eul0);
    eul_vel1 = fun_angle_velocity_to_eular(ang_vel_1, eul1);

    if strcmpi(interpolation_method, 'cubic')

        yaw = fun_poly3(t, 0, eul0(1), eul_vel0(1), T, eul1(1), eul_vel1(1), 0);
        yaw_vel = fun_poly3(t, 0, eul0(1), eul_vel0(1), T, eul1(1), eul_vel1(1), 1);

        pitch = fun_poly3(t, 0, eul0(2), eul_vel0(2), T, eul1(2), eul_vel1(2), 0);
        pitch_vel = fun_poly3(t, 0, eul0(2), eul_vel0(2), T, eul1(2), eul_vel1(2), 1);

        roll = fun_poly3(t, 0, eul0(3), eul_vel0(3), T, eul1(3), eul_vel1(3), 0);
        roll_vel = fun_poly3(t, 0, eul0(3), eul_vel0(3), T, eul1(3), eul_vel1(3), 1);

    elseif strcmpi(interpolation_method, 'quintic')

        yaw = fun_poly5(t, 0, eul0(1), eul_vel0(1), 0, T, eul1(1), eul_vel1(1), 0, 0);
        yaw_vel = fun_poly5(t, 0, eul0(1), eul_vel0(1), 0, T, eul1(1), eul_vel1(1), 0, 1);

        pitch = fun_poly5(t, 0, eul0(2), eul_vel0(2), 0, T, eul1(2), eul_vel1(2), 0, 0);
        pitch_vel = fun_poly5(t, 0, eul0(2), eul_vel0(2), 0, T, eul1(2), eul_vel1(2), 0, 1);

        roll = fun_poly5(t, 0, eul0(3), eul_vel0(3), 0, T, eul1(3), eul_vel1(3), 0, 0);
        roll_vel = fun_poly5(t, 0, eul0(3), eul_vel0(3), 0, T, eul1(3), eul_vel1(3), 0, 1);
    end

    eul = [yaw(:), pitch(:), roll(:)];
    eul_vel = [yaw_vel(:), pitch_vel(:), roll_vel(:)];

    quat = eul2quat(eul);
    ang_vel = zeros(size(eul_vel));

    for i = 1:size(eul_vel, 1)
        ang_vel(i, :) = fun_eular_velocity_to_angle(eul_vel(i, :), eul(i, :))';
    end

    t = t';
    pos = [x(:), y(:), z(:), quat];
    vel = [xd(:), yd(:), zd(:), ang_vel];

end
