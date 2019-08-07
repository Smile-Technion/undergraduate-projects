function mj_message(varargin)
%mjhx_message([text])
%   show text in simulator; no argument: clear

if nargin>0
    mjhx('message', varargin{1});
else
    mjhx('message');

end
