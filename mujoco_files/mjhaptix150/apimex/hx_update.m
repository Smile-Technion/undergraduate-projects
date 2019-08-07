function sensor = hx_update(command)
%sensor = hx_update(command)
%   send command, return sensor data
%   the simulator imposes a 1000/update_rate ms delay
%
%   command must have vector fields with size equal to motor_count:
%      ref_pos
%      ref_vel_max
%      gain_pos
%      gain_vel
%   and scalar fields specifying (as 1 or 0) which vectors to update:
%      ref_pos_enabled
%      ref_vel_max_enabled
%      gain_pos_enabled
%      gain_vel_enabled

sensor = mjhx('hx_update', command);

end
