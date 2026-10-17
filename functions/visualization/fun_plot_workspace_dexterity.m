function fun_plot_workspace_dexterity(workspace_data, base_tform, position_resolution)
    if nargin < 3
        position_resolution = 0.05;
    end

    figure;
    num_pos = length(workspace_data);
    point_data = zeros(num_pos, 3);
    color_data = zeros(num_pos, 3);

    fun_plot_frame(base_tform, 0.1, 3);
    hold on

    total_dexterity = 0;

    for i = 1:num_pos
        point_data(i, :) = workspace_data(i).position;
        color_data(i, :) = fun_color_mapping(workspace_data(i).dexterity);
        total_dexterity = total_dexterity + workspace_data(i).dexterity;
    end

    disp(['Total reachable positions: ', num2str(num_pos)])
    disp(['Total reachable volume: ', num2str(num_pos * position_resolution ^ 3)])
    disp(['Total dexterity = ', num2str(total_dexterity)])
    disp(['Average dexterity = ', num2str(total_dexterity / num_pos)])

    base_pos = base_tform(1:3, 4)';
    base_rotm = base_tform(1:3, 1:3);
    point_data = (base_rotm * point_data')' + base_pos;

    scatter3(point_data(:, 1), point_data(:, 2), point_data(:, 3), 100, color_data, 'filled');
    axis equal

    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    set(gca, 'FontSize', 15);

    set(gcf, 'color', [1 1 1]);
    disp('Figure: workspace_dexterity.');
    pause(0.1);

    figure;
    y = 0:0.01:1;
    x = zeros(1, length(y));

    for i = 1:length(x)
        color = fun_color_mapping(y(i));
        scatter(x(i), y(i), 50, color, 'filled');
        hold on
    end

    axis equal
    xlim([-0.1, 0.1])
    ylim([-0.1, 1.1])

    set(gcf, 'color', [1 1 1]);
    disp('Figure: color_bar.');
    pause(0.1);
