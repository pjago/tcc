%equivalent to global pwm(k) = u(k)
function writesim(d)
global u pwm k
%     pwm(k) = u(k);
    pwm(k) = d;
end