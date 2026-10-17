function fun_plot_workspace_point_cloud(workspace_data, position_resolution, vis_resolution_factor, base_tform)

    orientation_discretization_num = length(workspace_data(1).pose_reachability);

    vis_sphere_size = position_resolution / 5.0 * vis_resolution_factor;
    vis_point_size = sqrt(4 * pi * vis_sphere_size ^ 2 / orientation_discretization_num) / 2;

    point_data = fun_point_cloud_data(workspace_data, vis_sphere_size, base_tform);

    figure;
    fun_plot_frame(base_tform, 0.1, 3);
    hold on

    [x, y, z] = sphere();

    t0 = datetime('now');

    for i = 1:size(point_data, 1)

        if point_data(i, 4)
            surf(vis_point_size * x + point_data(i, 1), ...
                vis_point_size * y + point_data(i, 2), ...
                vis_point_size * z + point_data(i, 3), 'LineStyle', 'none', 'FaceColor', 'g');
        else
            surf(vis_point_size * x + point_data(i, 1), ...
                vis_point_size * y + point_data(i, 2), ...
                vis_point_size * z + point_data(i, 3), 'LineStyle', 'none', 'FaceColor', 'r');

        end

        hold on;
        fun_for_progress(size(point_data, 1), i, t0);

    end

    axis equal

    xlabel('x (m)');
    ylabel('y (m)');
    ylabel('z (m)');
    set(gca, 'FontSize', 15);

    set(gcf, 'color', [1 1 1]);
    disp('Figure: workspace.');
    pause(0.1);
