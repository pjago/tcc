function [kp, ti, td] = pplace(Gs, cs, varargin)
%PPLACE 2nd order pole placement
%   [kp, ti, td] = pplace(Gs, cs, 'PI+D')
%   [kp, td] = pplace(Gs, cs, 'PD')

if nargin == 2
   mode = 'PID';
else
   mode = varargin{1};
end

Gs = tf(Gs);
N = Gs.num{1};
D = Gs.den{1};
assert(size(D, 2) == 3, 'Gs must be a 2° order system!')
if ~isnumeric(cs)
    cs = cs.num{1};
    q = roots(cs)';
else
    q = roots(cs)';
end
assert(size(q, 2) == 3, 'cs must be 3° order! e.g. cs = (s³ + s² + 1)')

a1 = D(2);
a2 = D(3);
b1 = N(2);
b2 = N(3);

realq = find(~imag(q), 1);
compq = q(q ~= q(realq));
compeq = conv([1 -compq(1)], [1 -compq(2)]);
wo = sqrt(compeq(3));
zetawo = compeq(2)/2;
alphawo = -q(realq);
z = zetawo/wo;
a = alphawo/wo;

if strcmp(mode, 'PID')
    de = b2^3 - b1*b2^2*(a + 2*z)*wo + b1^2*b2*(1 + 2*a*z)*wo^2 - a*b1^3*wo^3;
    kp = (a2*b2^2 - a2*b1*b2*(a + 2*z)*wo - (b2 - a1*b1)*(b2*(1 + 2*a*z)*wo^2 + a*b1*wo^3))/de;
    ki = (-a1*b1*b2 + a2*b1^2 + b2^2)*a*wo^3/de;
    kd = (-a1*b2^2 + a2*b1*b2 + b2^2*(a + 2*z)*wo - b1*b2*wo^2*(1 + 2*a*z) + b1^2*a*wo^3)/de;
    ti = kp/ki;
    td = kd/kp;
elseif strcmp(mode, 'I+PD')
    assert(b1 == 0, 'I am lazy atm')
    ki = cs(4)/b2;
    kp = (cs(3) - a2)/b2;
    kd = (cs(2) - a1)/b2;
    ti = kp/ki;
    td = kd/kp;
end
    
end