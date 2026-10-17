function ang_vel = fun_eular_velocity_to_angle(eular_vel, eular_ang)
%FUN_EULAR_VELOCITY_TO_ANGLE  ZYX Euler rates -> world-frame angular velocity.
%
%   Inverse of FUN_ANGLE_VELOCITY_TO_EULAR. eular_vel is
%   [yaw_rate, pitch_rate, roll_rate] in rad/s and eular_ang is
%   [yaw, pitch, roll] in rad (the same convention as quat2eul/eul2quat with the
%   'ZYX' sequence). The returned ang_vel is expressed in the world (base) frame
%   in rad/s.
%
%   See also FUN_ANGLE_VELOCITY_TO_EULAR, FUN_POSE_INTERPOLATION.

    yaw = eular_ang(1);
    pitch = eular_ang(2);

    A = [cos(yaw) * cos(pitch), -sin(yaw), 0;
         sin(yaw) * cos(pitch),  cos(yaw), 0;
         -sin(pitch),           0,         1];

    rate_rpy = eular_vel([3, 2, 1]);      % [roll_rate; pitch_rate; yaw_rate]
    ang_vel = A * rate_rpy(:);

end
