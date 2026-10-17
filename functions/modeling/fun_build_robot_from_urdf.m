function robot = fun_build_robot_from_urdf(config)

    urdf_model = config.modeling.urdf_model;

    if ~exist(urdf_model, 'file')
        error('URDF model does not exist, please give the relative path to this librarby.');
    end

    robot = importrobot(urdf_model);
    robot.DataFormat = 'row';

    disp(['Load urdf model: "', urdf_model, '" successfully!']);
end
