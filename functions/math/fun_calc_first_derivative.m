function v = fun_calc_first_derivative(t, p)

    t = t(:);
    p = p(:);

    len = length(p);
    v = p;

    v(2:(end - 1)) = (p(3:end) - p(1:(end - 2))) ./ (t(3:end) - t(1:(end - 2)));

    v(1) = 2 * v(2) - v(3);
    v(len) = 2 * v(len - 1) - v(len - 2);

end
