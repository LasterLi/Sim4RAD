function robot = fun_define_gravity(robot, config)

    if config.modeling.gravity_direction == 0
        robot.Gravity = [0, 0, 0];
    elseif strcmpi(config.modeling.gravity_direction, '+x')
        robot.Gravity = [9.8, 0, 0];
    elseif strcmpi(config.modeling.gravity_direction, '+y')
        robot.Gravity = [0, 9.8, 0];
    elseif strcmpi(config.modeling.gravity_direction, '+z')
        robot.Gravity = [0, 0, 9.8];
    elseif strcmpi(config.modeling.gravity_direction, '-x')
        robot.Gravity = [-9.8, 0, 0];
    elseif strcmpi(config.modeling.gravity_direction, '-y')
        robot.Gravity = [0, -9.8, 0];
    elseif strcmpi(config.modeling.gravity_direction, '-z')
        robot.Gravity = [0, 0, -9.8];
    else
        error('Gravity_direction is defined mistakenly!')
    end

    disp(['Define gravity direction is "', config.modeling.gravity_direction, '".']);
end
