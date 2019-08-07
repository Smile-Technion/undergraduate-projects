function mj_set_geomsize(geomid, geomsize)
%mj_set_geomsize(geomid, geomsize)
%   set the size of a specified model geom
%   the geom type cannot be mesh
%   geomsize must have 3 elements (even if the geom type needs less)

mjhx('set_geomsize', geomid, geomsize);

end
