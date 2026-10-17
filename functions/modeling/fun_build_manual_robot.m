function robot = fun_build_manual_robot(config)

    robot = rigidBodyTree;

    % build the robot model
    for i = 1:config.manual_robot.num_links
        link_pose_name = ['Link', num2str(i), '_pose'];
        link_type_name = ['Link', num2str(i), '_type'];

        link_pose = config.manual_robot.(link_pose_name);
        R = eul2rotm([link_pose(6), link_pose(5), link_pose(4)] / 180 * pi);
        p = link_pose(1:3);
        T = [R, p'; [0, 0, 0, 1]];

        joint = rigidBodyJoint(['Joint', num2str(i)], config.manual_robot.(link_type_name));
        setFixedTransform(joint, T);
        Link = rigidBody(['Link', num2str(i)]);
        Link.Joint = joint;

        if i == 1
            addBody(robot, Link, 'base');
        else
            addBody(robot, Link, ['Link', num2str(i - 1)]);
        end

    end

    robot.DataFormat = 'row';

    % calc and define inertia and collision detection
    q = homeConfiguration(robot);
    volume = zeros(1, length(robot.Bodies) - 1);

    for i = 1:(length(robot.Bodies) - 1)
        T1 = getTransform(robot, q, robot.BodyNames{1, i});
        T2 = getTransform(robot, q, robot.BodyNames{1, i + 1});
        p1 = T1(1:3, 4);
        p2 = T2(1:3, 4);
        L = norm(p2 - p1);

        link_radius_name = ['Link', num2str(i), '_radius'];
        volume(i) = pi * (config.manual_robot.(link_radius_name)) ^ 2 * L;
    end

    rho = config.manual_robot.estimated_total_mass / sum(volume);

    for i = 1:(length(robot.Bodies) - 1)
        link_radius_name = ['Link', num2str(i), '_radius'];

        T = getTransform(robot, q, robot.BodyNames{1, i + 1}, robot.BodyNames{1, i});

        vec = T(1:3, 4);
        L = norm(vec);
        mid_point = vec / 2;

        if L > 0
            R = fun_vector2rotm(vec);

            Izz = 0.5 * rho * volume(i) * (config.manual_robot.(link_radius_name)) ^ 2;
            Ixx = rho * volume(i) / 12 * (3 * (config.manual_robot.(link_radius_name)) ^ 2 +L ^ 2);
            Iyy = Ixx;
            I0 = [Ixx, 0, 0;
                  0, Iyy, 0;
                  0, 0, Izz];
            I = R * I0 * R' + rho * volume(i) * (mid_point' * mid_point * eye(3) - mid_point * mid_point');

            robot.Bodies{1, i}.Mass = rho * volume(i);
            robot.Bodies{1, i}.CenterOfMass = mid_point';
            robot.Bodies{1, i}.Inertia = [I(1, 1), I(2, 2), I(3, 3), 0, 0, 0];

            Geo = collisionCylinder(config.manual_robot.(link_radius_name), L);
            Geo.Pose = [R, mid_point; 0, 0, 0, 1];
            addCollision(robot.Bodies{1, i}, Geo);
        else
            robot.Bodies{1, i}.Mass = 0;
            robot.Bodies{1, i}.CenterOfMass = [0, 0, 0];
            robot.Bodies{1, i}.Inertia = [0, 0, 0, 0, 0, 0];
        end

    end

    robot.Bodies{1, end}.Mass = 0;
    robot.Bodies{1, end}.CenterOfMass = [0, 0, 0];
    robot.Bodies{1, end}.Inertia = [0, 0, 0, 0, 0, 0];

    disp('Build manual robot model successfully!');

end
