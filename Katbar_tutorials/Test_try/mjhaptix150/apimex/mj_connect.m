function mj_connect(varargin)
%mj_connect([host])
%   connect to specified host
%   empty string or missing argument: local host

if nargin>0
    mjhx('connect', varargin{1});
else
    mjhx('connect');

end
