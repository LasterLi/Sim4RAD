clear
clc

global error_threshold;
error_threshold = 1e-3;

% Dynamics simulation: obtain the following results based on the defined trajectory paramters.
% (1) joint motion data: [t,q,qd,qdd];
% (2) joint drive requirement: vel, tau_motor, tau_out, power
% (3) end state: pos, quat, linear velocity, angular velocity, linear acceleration, angular acceleration.

addpath(genpath(pwd));

% Read config:
% Please fully define "config_modeling.yaml" (necessary) and "config_manual_robot.yaml" (optional).
% Please fully define "config_dynamics.yaml" (necessary).
config = fun_read_config();

% Build robot model
robot = fun_build_robot(config);

% Generate primitive trajectory data based on config file.
[t, q, qd, qdd] = fun_trajectory_generation(robot, config);

% Update the time scale factor to satisfy the trajectory constraint.
[t, q, qd, qdd] = fun_trajectory_constraint(robot, t, q, qd, qdd, config);

% Calculate the end effector trajectory
[pos, vel, acc, linear_vel, ang_vel, linear_acc, ang_acc] = fun_trajectory_fk(robot, t, q, qd, qdd);

% Get the joint drive requirement by dynamics simulation
if config.dynamics.consider_transmission_loss
    disp('Transmission loss is considered!');
end

[transmission, joint_capability] = fun_get_joint_dynamics_capability(config.dynamics);

% The end load is already added as a rigid body in the model (see fun_add_end_load),
% so its gravity and inertia are included. Set the external wrench to zero here,
% unless an additional contact/task force is required, e.g. [0, 0, 0, 0, 0, 60].
end_wrench = zeros(1, 6);
[tau_motor, tau_out, power_motor, power_out] = fun_trajectory_inverse_dynamics(robot, t, q, qd, qdd, transmission, end_wrench);

% plot simulation results.
fun_plot_joint_trajectory(t, q, qd, qdd, true);
fun_plot_ee_trajectory(t, pos, vel, acc, linear_vel, ang_vel, linear_acc, ang_acc);
fun_plot_joint_effort(t, tau_motor, tau_out, power_motor, power_out, joint_capability, false);
fun_plot_joint_vel_effort(t, tau_motor, tau_out, qd, transmission, joint_capability);

% generate gif of the robot motion.
% figure;
% fun_generate_gif(robot, t, q, pos, config);
