clear
clc

addpath(genpath(pwd));

% Read config:
% Please fully define "config_modeling.yaml" (necessary) and "config_manual_robot.yaml" (optional).
config = fun_read_config();

% Build robot model
robot = fun_build_robot(config);

q = homeConfiguration(robot);
% q=randomConfiguration(robot);
% q=[0,90,0,90,0,90,0]*pi/180;

fun_show_robot(robot, q, config);
disp(['Plot the robot at configuration: ', num2str(q)]);

tot_mass = fun_calc_robot_mass(robot, config);
disp(['Total weight of the robot (without fixed base and end load) is ', num2str(tot_mass), ' kg.']);
