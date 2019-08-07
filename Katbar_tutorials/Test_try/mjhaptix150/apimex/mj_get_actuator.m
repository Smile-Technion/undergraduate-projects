function actuator = mj_get_actuator()
%mj_get_actuator()
%   return actuator lengths, velocities and forces

actuator = mjhx('get_actuator');

end
