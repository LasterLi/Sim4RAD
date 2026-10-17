function [mean_x, mean_y, std_x, std_y] = fun_scatter_data_statistics(x, y)

    [sort_x, sort_idx] = sort(x);
    sort_y = y(sort_idx);

    % scatter(sort_x,sort_y, 'MarkerEdgeColor', [0.7,0.7,0.7], 'MarkerFaceColor', 'none');
    % hold on

    num_group = 8;
    num_each_group = round(length(sort_x) / num_group);

    mean_x = zeros(1, num_group);
    mean_y = zeros(1, num_group);
    std_x = zeros(1, num_group);
    std_y = zeros(1, num_group);

    for i = 1:num_group

        if i < num_group
            data_x = sort_x((i - 1) * num_each_group + 1:i * num_each_group);
            data_y = sort_y((i - 1) * num_each_group + 1:i * num_each_group);
        else
            data_x = sort_x((i - 1) * num_each_group + 1:end);
            data_y = sort_y((i - 1) * num_each_group + 1:end);
        end

        mean_x(i) = mean(data_x);
        mean_y(i) = mean(data_y);
        std_x(i) = std(data_x);
        std_y(i) = std(data_y);
    end

end
