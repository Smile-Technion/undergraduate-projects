function id = mj_name2id(type, name)
%d = mj_name2id(type, name)
%   return id of object of specified type and name
%   type can be: 'body', 'geom', 'site', 'joint', 'tendon', 'sensor', 'actuator', 'equality', 'material'

id = mjhx('name2id', type, name);

end
