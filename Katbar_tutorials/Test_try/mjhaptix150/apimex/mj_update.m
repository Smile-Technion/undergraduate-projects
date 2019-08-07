function sensor = mj_update(control)
%sensor = mj_update(control)
%   set control, return sensor data; see output of mj_get_control

sensor = mjhx('update', control);

end
