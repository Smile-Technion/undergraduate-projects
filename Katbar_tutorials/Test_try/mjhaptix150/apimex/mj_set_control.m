function mj_set_control(control)
%mj_set_control(control)
%   set simulator control; see output of mj_get_control

mjhx('set_control', control);

end
