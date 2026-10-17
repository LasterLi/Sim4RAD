function [q, qd, flag] = fun_ik(robot, pos, vel, q_guess, error_threshold)

    flag = 1;

    ik = inverseKinematics('RigidBodyTree', robot);
    LastLink = robot.BodyNames{1, end};

    R = quat2rotm(pos(4:7));
    T = [R, pos(1:3)'; 0, 0, 0, 1];

    weights = [1, 1, 1, 1, 1, 1];

    [q, solnInfo] = ik(LastLink, T, weights, q_guess);

    if solnInfo.PoseErrorNorm > error_threshold
        warning(['End pose cannot reach! Error is ', num2str(solnInfo.PoseErrorNorm)]);
        flag = 0;
    end

    dof = length(q_guess);

    if isempty(vel)
        qd = zeros(1, dof);
        return;
    else
        jac = geometricJacobian(robot, q, LastLink);
        vel = [vel(4:6), vel(1:3)];
        qd = pinv(jac) * vel';
        qd = qd';

        vel_test = jac * qd';
        err = norm(vel_test - vel');

        if err > error_threshold
            warning('Pose is close to singularity. End velocity cannot reach!');
            flag = 0;
        end

    end

end
