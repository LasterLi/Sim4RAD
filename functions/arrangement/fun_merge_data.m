function [merge_data, idx] = fun_merge_data(ori_data, dim, err)
    % dim=1, merge_same_row;
    % dim=2, merge_same_column;

    if dim == 1
        [merge_data, idx] = fun_merge_same_row(ori_data, err);
    elseif dim == 2
        [merge_data, idx] = fun_merge_same_row(ori_data', err);
        merge_data = merge_data';
    else
        error('Wrong imput: dim should be 1 or 2 for fun_merge_data(ori_data,dim,err).');
    end

end

function [merge_data, idx] = fun_merge_same_row(ori_data, err)

    merge_data = ori_data(1, :);
    idx = 1;

    for i = 2:size(ori_data, 1)
        kk = 0;

        for k = 1:size(merge_data, 1)

            if norm(ori_data(i, :) - merge_data(k, :)) < err
                kk = k;
                break
            end

        end

        if kk == 0
            merge_data = [merge_data; ori_data(i, :)];
            idx = [idx; i];
        end

    end

end
