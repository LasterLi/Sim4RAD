function vectors = fun_orientation_discretization(num)

    golden_ratio = (1 + sqrt(5)) / 2;
    angle_increment = 2 * pi * golden_ratio;

    vectors = zeros(3, num);

    for i = 1:num
        t = (i - 1) / (num - 1);
        inclination = acos(1 - 2 * t);
        azimuth = angle_increment * (i - 1);

        x = sin(inclination) * cos(azimuth);
        y = sin(inclination) * sin(azimuth);
        z = cos(inclination);

        vectors(:, i) = [x, y, z]' / norm([x, y, z]);
    end

end
