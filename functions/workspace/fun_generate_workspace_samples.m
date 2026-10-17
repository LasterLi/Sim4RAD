function samples = fun_generate_workspace_samples(config)

    position_resolution = config.position_resolution;
    space_range = fun_get_space_range(config);
    arm_length = config.arm_length;

    disp('Search space range: ');
    disp(['Lower limits: ', num2str(space_range.lower)]);
    disp(['Upper limits: ', num2str(space_range.upper)]);
    disp('----------------------------------------------------------------------------------------------------');

    small_offset = position_resolution * 0.001;
    p0 = ceil((space_range.lower - small_offset) / position_resolution) * position_resolution;
    p1 = space_range.upper + small_offset;
    x = p0(1):position_resolution:p1(1);
    y = p0(2):position_resolution:p1(2);
    z = p0(3):position_resolution:p1(3);

    sample_points = fun_traversal({x, y, z});
    norm_sample_points = sqrt(sample_points(:, 1) .^ 2 + sample_points(:, 2) .^ 2 + sample_points(:, 3) .^ 2);
    sample_points = sample_points(norm_sample_points <= arm_length, :);
    num_sample_points = size(sample_points, 1);

    disp(['Num of sampling positions is: ', num2str(num_sample_points)]);

    orientation_discretization_num = config.orientation_discretization_num;
    vectors = fun_orientation_discretization(orientation_discretization_num);

    disp(['Num of sampling orientations is: ', num2str(orientation_discretization_num)]);
    disp(['Num of total samples is: ', num2str(num_sample_points * orientation_discretization_num)]);
    disp('----------------------------------------------------------------------------------------------------');

    samples.points = sample_points';
    samples.num_pos = num_sample_points;
    samples.directions = vectors;
    samples.num_dir = orientation_discretization_num;

end
