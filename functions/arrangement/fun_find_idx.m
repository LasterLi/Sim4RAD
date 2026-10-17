function [idx, dis] = fun_find_idx(data, vec)

    dim = size(data, 1);
    num = size(data, 2);

    err = data - vec;

    err_norm = err(1, :) .^ 2;

    for i = 2:dim
        err_norm = err_norm + err(i, :) .^ 2;
    end

    err_norm = sqrt(err_norm);

    [dis, idx] = min(err_norm);
end
