function p = fun_poly3(t, t0, p0, v0, t1, p1, v1, order)

    A = [t0 ^ 3, t0 ^ 2, t0, 1;
         3 * t0 ^ 2, 2 * t0, 1, 0;
         t1 ^ 3, t1 ^ 2, t1, 1;
         3 * t1 ^ 2, 2 * t1, 1, 0];
    B = [p0, v0, p1, v1]';
    a = pinv(A) * B;

    if order == 0
        p = a(1) * t .^ 3 + a(2) * t .^ 2 + a(3) * t + a(4);
    elseif order == 1
        p = 3 * a(1) * t .^ 2 + 2 * a(2) * t + a(3);
    else
        p = 6 * a(1) * t + 2 * a(2);
    end

end
