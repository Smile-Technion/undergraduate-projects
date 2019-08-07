function mj_reset(varargin)
%mjhx_reset([key])
%  reset simulation to specified key frame
%  no argument or key = -1: reset to reference configuration

if nargin>0
    mjhx('reset', varargin{1});
else
    mjhx('reset');
    
end
