function [kp, Tf, td] = pdfplace(Gs, varargin)
%PDFPLACE 2nd order poles placement
%   [kp, Tf, td,] = pdfplace(Gs, poles)
%   [kp, Tf, td, poles] = pdfplace(Gs, overshoot, peak_time)

% todo: make trisectrix
% http://ieeecss.org/CSM/library/2009/oct09/NyquistShapes.pdf

Gs = tf(Gs);
N = Gs.num{1};
D = Gs.den{1};
assert(size(D, 2) == 3 && N(1) == 0 && N(2) == 0, 'pass me a 2° lpf!')
a = D(1)/N(3);
b = D(2)/N(3);
c = D(3)/N(3);
p1 = (-b + sqrt(b^2 - 4*a*c))/(2*a);
p2 = (-b - sqrt(b^2 - 4*a*c))/(2*a);

if nargin == 3
    shoot = varargin{1};
    tpeak = varargin{2};
    psi = log(shoot)/sqrt(pi + log(shoot)^2);
    wn = pi/(tpeak*sqrt(1 - psi^2));
    x1 = wn*(-psi + 1j*sqrt(1 - psi^2));
else
    %we should assign by specifing the sensitivity dcgain
    if nargin == 1
        psi = 0.99; %most damping at sensible frequency
    else
        psi = varargin{1};
    end
    [~, wp] = getPeakGain(Gs/(1+Gs));
    x1 = -wp*(psi + 1j*sqrt(1 - psi^2));
    x2 = x1';
    den = conv([1 -x1], [1 -x2]);
    wn = wp;
    psi = den(2)/(2*wn);
    shoot = exp(-psi*pi/(sqrt(1 - psi^2)));
    tpeak = pi/(wn*sqrt(1 - psi^2)); %wrong formula?
    tset2 = -log(0.02*sqrt(1 - psi^2))/(psi*wn*log(exp(1)));
    disp(['wn: ' num2str(wn) ' psi: ' num2str(psi)])
    disp(['Ms: ' num2str(shoot) ' tp: ' num2str(tpeak) ' ts: ' num2str(tset2)])
end
%phase shift frequency
% the peak in the sensivity just happens to be at same place as the 
% peak on the complementary sensitivity for a 2nd order Tf
% either way, this should reduce the ring effect
[~, wp] = getPeakGain(Gs/(1+Gs));
wshift = wp; % shift to counter the rise of sensitivity
% ideally the placement should occur for wp, so the root locus should
% pass across that point having the desired damping. that way there is
% margin to increase (or decrease) the steady error by changing the gain,
% keeping the same or better damping.
phi = angle(x1-p1) + angle(x1-p2) - pi;
R = real(x1);
I = imag(x1);
A = (I - R*tan(phi));
B = tan(phi)*(R^2 + I^2 + wshift^2);
C = -wshift^2*(I + R*tan(phi));
delta = B^2 - 4*A*C;
ps = [(-B+sqrt(delta))/(2*A), (-B-sqrt(delta))/(2*A)];
p3 = min(ps); % tenta eliminar a solução resulta em compesador de atraso!
assert(abs(p3) > wshift, 'Td would be negative. aka lag compesator!')
thetaz = angle(x1-p1) + angle(x1-p2) + angle(x1-p3) - pi; %angle criterion
z1 = (real(x1) - imag(x1)/tan(thetaz));

Tf = -1/p3;
td = -Tf - 1/z1;
kp = a*(abs(x1-p1)*abs(x1-p2)*abs(x1-p3))/abs(x1-z1)*Tf/(Tf+td); %gain criterion

end