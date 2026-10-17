function R = fun_vector2rotm(vec)
    % based on a vector to obtain a rotm. Solution is not unique, only return one.

    vec = vec(:)';

    z_vec = vec / norm(vec);

    if z_vec(1) ~= 0 || z_vec(2) ~= 0
        x_vec = [-z_vec(2), z_vec(1), 0];
    else
        x_vec = [0, -z_vec(3), z_vec(2)];
    end

    x_vec = x_vec / norm(x_vec);
    y_vec = cross(z_vec, x_vec);
    y_vec = y_vec / norm(y_vec);

    R = [x_vec', y_vec', z_vec'];
end
