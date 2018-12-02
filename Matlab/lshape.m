function [kp, ti, td] = lshape(Gs, varargin)
%WIP: PID (done!), PD (not!)
%LSHAPE zero placement on dominant pole(s)
%   [kp, td, tf] = lshape(Gs, settling_time)
%   TODO1: relacao entre settling_time e overshoot
%   TODO2: place pole outside of zero. (compensador avanco de fase)
%   OBJ1: assentamento 2% em 1 s, com erro de regime nulo ^^^^
%   OBJ2: escolha da frequencia de cruzamento entre o sigma de T e S
%
% Ex: Y1 = 10*feedback(Gs*C1, 1)      Y2 = feedback(Gs*C2, 1)
%
% Gs =
%  
%                 97
%   --------------------------------
%   (s+2.501) (s^2 + 1.714s + 21.05)
%
% C1 =
%  
%   0.0069095 (s^2 + 1.714s + 21.05)
%   --------------------------------
%              (s+4.822)
%
% C2 =
%  
%   0.016349 (s^2 + 1.714s + 21.05)
%   -------------------------------
%                  s

Gs = zpk(Gs);
poles = Gs.p{1};
assert(~sum(real(poles) > 0), 'cant invert unstable poles!')
if nargout == 0
    disp('LSHAPE loop shape target')
end
[~, rank] = sort(real(poles), 'descend');
poles = poles(rank);
if length(poles) == 2
    p = poles(2:end);
    z = [poles(1); poles(1)];
else
    p = poles(3:end);
    z = poles(1:2);
end
if isempty(p)
    ts = varargin{1};
    %settling happens at ~4*tau
    k = -log(0.02)/ts;
    if nargout == 0
        disp(['wn: ' num2str(k) ' zeta: NaN'])
        disp(['Ms: 0' ' tp: Inf' ' ts: ' num2str(ts)])
    end
else
    %gain for 2nd order rise time
    assert(length(p) == 1, 'Gs must have at most 3 poles!')
    d = -real(p(1)); % zeta = d/(2*wn)
    if nargin == 1 % critically damped
        zeta = 1;
        wn = d/2;
    else
        ts = varargin{1};
        if ts < 5.8335/(d/2) % underdamped
            zeta = sqrt(1 - exp(-ts*d)/4e-4);
            assert(~imag(zeta), ['Settle time has to be > ' num2str(-2*log(0.02)/d)])
            wn = d/(2*zeta);
        else % overdamped
        %https://electronics.stackexchange.com/questions/296567/over-and-critically-damped-systems-settling-time
            syms zeta
            assume(zeta, 'real'); %#ok<NODEF>
            assume(zeta >= 1);
            wn = d/(2*zeta);
            K = zeta/sqrt(zeta^2 - 1);
            D = wn*sqrt(zeta^2 - 1);
            zeta = eval(solve(exp(-K*D*ts)*(cosh(D*ts) + K*sinh(D*ts)) - 0.02));
            wn = d/(2*zeta);
        end
    end
    k = wn^2;
    %DISP
    if zeta < 1.0
        Ms = exp(-zeta*pi/(sqrt(1 - zeta^2)));
        ts = -log(0.02*sqrt(1 - zeta^2))/(zeta*wn);
        tp = pi/(wn*sqrt(1 - zeta^2));
        if nargout == 0
            disp(['wn: ' num2str(wn) ' zeta: ' num2str(zeta)])
            disp(['Ms: ' num2str(Ms) ' tp: ' num2str(tp) ' ts: ' num2str(ts)])
        end
    else
        try
            C = pid(zpk(z, 0, k/Gs.k));
            info = stepinfo(feedback(C*Gs, 1));
            if nargout == 0
                disp(['wn: ' num2str(wn) ' zeta: ' num2str(zeta)])
                disp(['Ms: 0' ' tp: Inf' ' ts: ' num2str(info.SettlingTime)])
            end
        catch
        end
    end
end
try
    C = pid(zpk(z, 0, k/Gs.k)); % PID
    kp = C.kp;
    ti = kp/C.ki;
    td = C.kd/kp;
catch ME
    C = zpk(z, 0, k/Gs.k)
    throw(ME)
end
end