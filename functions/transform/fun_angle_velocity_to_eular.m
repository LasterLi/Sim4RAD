function eular_vel = fun_angle_velocity_to_eular(ang_vel, eular_ang)
%FUN_ANGLE_VELOCITY_TO_EULAR  World-frame angular velocity -> ZYX Euler rates.
%
%   Eular angles follow the same convention as quat2eul/eul2quat with the 'ZYX'
%   sequence, i.e. eular_ang = [yaw, pitch, roll] in rad. ang_vel is the angular
%   velocity expressed in the world (base) frame, as specified for Cartesian
%   trajectories in config_dynamics.yaml, in rad/s. The returned eular_vel is
%   [yaw_rate, pitch_rate, roll_rate] in rad/s.
%
%   The mapping used is
%       w = A * [roll_rate; pitch_rate; yaw_rate],
%       A = [ cos(yaw)cos(pitch), -sin(yaw), 0 ;
%             sin(yaw)cos(pitch),  cos(yaw), 0 ;
%            -sin(pitch),         0,         1 ],
%   so that the Euler angles, their rates and the angular velocity describe the
%   same motion. The pseudo-inverse is used instead of an explicit inverse so
%   that the singular configuration (pitch = +-90 deg) does not raise an error.
%
%   See also FUN_EULAR_VELOCITY_TO_ANGLE, FUN_POSE_INTERPOLATION.

    yaw = eular_ang(1);
    pitch = eular_ang(2);

    A = [cos(yaw) * cos(pitch), -sin(yaw), 0;
         sin(yaw) * cos(pitch),  cos(yaw), 0;
         -sin(pitch),           0,         1];

    rate_rpy = pinv(A) * ang_vel(:);     % [roll_rate; pitch_rate; yaw_rate]

    eular_vel = rate_rpy([3, 2, 1]);     % [yaw_rate; pitch_rate; roll_rate]

end
