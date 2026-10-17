function fun_generate_gif(robot, t, q, pos, config)

    disp('Plot the robot pose in time series to generate a gif of the robot motion.');

    % build a figure
    figure();

    dt = t(2) - t(1);
    %Frame interval time
    delay_time = 0.05;

    if dt >= delay_time
        delay_time = dt;
    end

    date_time = datestr(now, 'yyyy-mm-dd-HH-MM');
    fila_name = ['gif\animation_', date_time, '.gif'];
    t0 = datetime('now');

    for i = 1:round(delay_time / dt):length(t)
        clf

        fun_show_robot(robot, q(i, :), config)

        plot3(pos(1:i, 1), pos(1:i, 2), pos(1:i, 3), 'k', 'LineWidth', 2);
        axis equal
        axis([-1.5, 1.5, -1.5, 1.5, -1.5, 1.5])
        set(gcf, 'color', [1, 1, 1])

        pause(0.1);

        % 将当前帧保存为 GIF 动画
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        if i == 1
            imwrite(imind, cm, fila_name, 'gif', 'Loopcount', inf, 'DelayTime', delay_time);
        else
            imwrite(imind, cm, fila_name, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
        end

        fun_for_progress(length(1:round(delay_time / dt):length(t)), (i - 1) / round(delay_time / dt) + 1, t0);
    end

    disp(['Generate a fig animation: ', fila_name]);
    disp('----------------------------------------------------------------------------------------------------');
end
