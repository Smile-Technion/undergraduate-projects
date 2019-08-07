function rgba = mj_get_rgba(type, id)
%rgba = mj_get_rgba(type, id)
%   get rgba of model element with specified type and id
%   type must be one of: 'geom', 'site', 'tendon', 'material'

rgba = mjhx('get_rgba', type, id);

end
