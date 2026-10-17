function singularity = fun_singularity(robot, q);
    LastLink = robot.BodyNames{1, end};
    J = geometricJacobian(robot, q, LastLink);
    singularity = sqrt(det(J * J'));
end
