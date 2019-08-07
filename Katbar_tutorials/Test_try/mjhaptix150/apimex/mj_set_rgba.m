function mj_set_rgba(type, id, rgba)
%mj_set_rgba(type, id, rgba)
%   set rgba of model element with specified type and id
%   type can be: 'geom', 'site', 'tendon', 'material'
%   rgba must have size 4

mjhx('set_rgba', type, id, rgba);

end
