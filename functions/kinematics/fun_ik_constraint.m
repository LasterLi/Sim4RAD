function [q, flag] = fun_ik_constraint(gik, LastLink, pos, direction, q_guess, error_threshold)

    flag = 1;

    % LastLink = robot.BodyNames{1, end};

    positionConst = constraintPositionTarget(LastLink);
    positionConst.TargetPosition = pos;

    aimConst = constraintAiming(LastLink);
    aimConst.TargetPoint = pos + direction;

    [q, solInfo] = gik(q_guess, positionConst, aimConst);

    [err_pos, err_dir] = solInfo.ConstraintViolations.Violation;

    % Iterations = solInfo.Iterations
    % NumRandomRestarts = solInfo.NumRandomRestarts

    if err_pos > error_threshold || err_dir > error_threshold
        flag = 0;
    end

end
