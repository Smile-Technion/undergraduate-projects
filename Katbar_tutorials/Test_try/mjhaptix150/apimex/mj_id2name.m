function name = mj_id2name(type, id)
%did = mj_id2name(type, id)
%   return name of object of specified type and id
%   type can be: 'body', 'geom', 'site', 'joint', 'tendon', 'sensor', 'actuator', 'equality', 'material'

name = mjhx('id2name', type, id);

end
