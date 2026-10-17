function fun_show_robot(robot, q, config)

    if config.modeling.using_urdf_model
        show(robot, q);
    else
        show(robot, q, 'Collisions', 'on');
    end

    hold on;
end
