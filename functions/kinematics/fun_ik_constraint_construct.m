function gik = fun_ik_constraint_construct(robot)

    gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'position', 'aiming'});
    % gik.SolverAlgorithm = 'LevenbergMarquardt';
    gik.SolverParameters.MaxIterations = 200; %1500
    % gik.SolverParameters.MaxTime = 100;   % 10
    gik.SolverParameters.GradientTolerance = 1e-4; %1e-7
    gik.SolverParameters.SolutionTolerance = 1e-4; %1e-6
    % gik.SolverParameters.EnforceJointLimits = 1;
    % gik.SolverParameters.AllowRandomRestarts = 1;
    gik.SolverParameters.StepTolerance = 1e-4; %1e-14

end
