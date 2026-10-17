function robot = fun_add_end_load(robot, config)

    % add end load
    if config.modeling.add_end_load
        I = config.modeling.end_load_inertia;

        LastLink = robot.BodyNames{1, end};

        eul = config.modeling.end_load_pos(4:6);
        R_end = eul2rotm([eul(3), eul(2), eul(1)]);
        T_end = [R_end, [config.modeling.end_load_pos(1:3)]'; 0, 0, 0, 1];
        J_end = rigidBodyJoint('J_end', 'fixed');
        setFixedTransform(J_end, T_end);
        Link_End = rigidBody('Link_End');
        Link_End.Joint = J_end;
        Link_End.Mass = config.modeling.end_load;
        Link_End.CenterOfMass = [0, 0, 0];
        Link_End.Inertia = [I, I, I, 0, 0, 0];
        addBody(robot, Link_End, LastLink);

        disp(['Add end load ', num2str(Link_End.Mass), ' kg!']);
    end

end
