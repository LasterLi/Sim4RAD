function tot_mass = fun_calc_robot_mass(robot, config)

    tot_mass = 0;

    if config.modeling.add_end_load

        for i = 1:length(robot.Bodies) - 1
            tot_mass = tot_mass + robot.Bodies{1, i}.Mass;
        end

    else

        for i = 1:length(robot.Bodies)
            tot_mass = tot_mass + robot.Bodies{1, i}.Mass;
        end

    end

end
