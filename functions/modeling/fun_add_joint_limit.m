function [robot, config] = fun_add_joint_limit(robot, config)

    if config.modeling.override_joint_limit || ~config.modeling.using_urdf_model
        i = 1; % link index
        j = 1; % dof index

        while true

            if strcmpi(robot.Bodies{1, i}.Joint.Type, 'revolute')
                robot.Bodies{1, i}.Joint.PositionLimits = [config.modeling.joint_lower_limit(j), config.modeling.joint_upper_limit(j)];
                i = i + 1;
                j = j + 1;
            elseif strcmpi(robot.Bodies{1, i}.Joint.Type, 'fixed')
                i = i + 1;
            else
                error('Only supported joint type is revolute or fixed, please check');
            end

            if i > length(robot.Bodies) || j > length(config.modeling.joint_upper_limit)
                break
            end

        end

        disp('Define joint range limits by user in config file!');
    else
        i = 1; % link index
        j = 1; % dof index

        while true

            if strcmpi(robot.Bodies{1, i}.Joint.Type, 'revolute')
                config.modeling.joint_lower_limit(j) = robot.Bodies{1, i}.Joint.PositionLimits(1);
                config.modeling.joint_upper_limit(j) = robot.Bodies{1, i}.Joint.PositionLimits(2);
                i = i + 1;
                j = j + 1;
            elseif strcmpi(robot.Bodies{1, i}.Joint.Type, 'fixed')
                i = i + 1;
            else
                error('Only supported joint type is revolute or fixed, please check');
            end

            if i > length(robot.Bodies)
                break
            end

        end

        disp('Use joint range limits defined in URDF!');
    end

    disp(['Joint lower limits: ', num2str(config.modeling.joint_lower_limit)]);
    disp(['Joint upper limits: ', num2str(config.modeling.joint_upper_limit)]);
end
