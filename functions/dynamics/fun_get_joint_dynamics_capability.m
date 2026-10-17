function [transmission, joint_capability] = fun_get_joint_dynamics_capability(config)

    joint_capability.joint_peak_torque = config.joint_peak_torque;
    joint_capability.joint_peak_velocity = config.joint_peak_velocity;
    joint_capability.motor_peak_torque = config.joint_peak_torque;
    joint_capability.motor_peak_velocity = config.joint_peak_velocity;

    if config.consider_transmission_loss
        joint_capability.joint_peak_torque = joint_capability.joint_peak_torque .* config.transmission_efficiency;
        joint_capability.joint_peak_velocity = joint_capability.joint_peak_velocity;
        joint_capability.motor_peak_torque = joint_capability.motor_peak_torque ./ config.gear_ratio;
        joint_capability.motor_peak_velocity = joint_capability.motor_peak_velocity .* config.gear_ratio;
    end

    transmission.consider = config.consider_transmission_loss;
    transmission.motor_inertia = config.motor_inertia;
    transmission.gear_ratio = config.gear_ratio;
    transmission.efficiency = config.transmission_efficiency;
