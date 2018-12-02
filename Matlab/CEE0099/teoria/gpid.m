function g = gpid(kp, ti, varargin)
%GPID Polynome of PID transfer gains
%   g = gpid(kp, ti, td, T)

if nargin == 3
    td = 0;
    T = varargin{1};
elseif nargin == 4
    td = varargin{1};
    T = varargin{2};
end

i = T/(2*ti);
d = td/T;

g0 = (i + d + 1)*kp;
g1 = (i - 2*d - 1)*kp;
g2 = (d)*kp;

if nargin == 3
    g = [g1, g0];
else
    g = [g2, g1, g0];
end
   
end