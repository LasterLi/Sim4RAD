function [robot, config] = fun_build_robot(config)
    disp('----------------------------------------------------------------------------------------------------');

    if config.modeling.using_urdf_model
        robot = fun_build_robot_from_urdf(config);
    else
        robot = fun_build_manual_robot(config);
    end

    dof = length(homeConfiguration(robot));
    disp(['The robot arm has ', num2str(dof), ' dof.']);

    robot = fun_define_gravity(robot, config);
    robot = fun_add_end_load(robot, config);
    disp('----------------------------------------------------------------------------------------------------');

    % fun_add_joint_limit returns the config as well: when the limits are taken
    % from the URDF (override_joint_limit = false), they are written into the
    % config here, and the caller needs this updated copy to use the same limits
    % in the subsequent analyses.
    [robot, config] = fun_add_joint_limit(robot, config);
    disp('----------------------------------------------------------------------------------------------------');

end
