function data = fun_traversal(groups)
    % There are N groups, each group has different length n_i. groups is a cell of 1xN.
    % For traversal, data has [\prod_1^N n_i] row, and in each line, the i^th element is from i^th group.

    num_groups = length(groups);
    length_groups = zeros(num_groups, 1);
    num_all = 1;

    for i = 1:num_groups
        length_groups(i) = length(groups{i});
        num_all = num_all * length_groups(i);
    end

    % Generate traversal index.
    index_group = ones(num_all, num_groups);

    for i = 2:num_all
        idx = index_group(i - 1, :);
        idx(end) = idx(end) + 1;

        for j = num_groups:-1:2

            if idx(j) > length_groups(j)
                idx(j) = 1;
                idx(j - 1) = idx(j - 1) + 1;
            end

        end

        index_group(i, :) = idx;
    end

    % generate traversal data
    data = zeros(num_all, num_groups);

    for i = 1:num_all

        for j = 1:num_groups
            data(i, j) = groups{j}(index_group(i, j));
        end

    end

end
