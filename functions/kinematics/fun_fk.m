function [pos, vel] = fun_fk(robot, q, qd)

    LastLink = robot.BodyNames{1, end};

    T = getTransform(robot, q, LastLink);
    pos = [T(1:3, 4)', rotm2quat(T(1:3, 1:3))];

    if nargin < 3 || isempty(qd)
        vel = [];
        return;
    else
        jac = geometricJacobian(robot, q, LastLink);
        vel = jac * qd';
        vel = [vel(4:6); vel(1:3)]';
    end

end
