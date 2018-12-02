function [C1, C2] = ctrl_pplace(G, type, poly_or_wn)
%CTRL Calculates the 2DOF controllers for poles placement 
%   type of controller can be one of: 'I+PD', 'CMP+D'
%   number of poles must comply with plant and ctrl order

s = tf('s');
G = zpk(G);
gp = -G.p{1};
ga = 1;
for i = 1:length(gp)
    ga = conv(ga, [1 gp(i)]);
end

assert(size(gp, 1) == 3, 'ctrl expects G to have 3 poles')
assert(size(G.z{1}, 1) == 0, 'ctrl expects G with no zeros')
assert(~sum(real(gp) < 0), 'ctrl expects no unstable poles')
[~, rank] = sort(real(gp));
gp = gp(rank);

%TODO: have poly choose as one of selected groups (e.g. best ITAE)
if length(poly_or_wn) == 1 && ~strcmp(type, 'PI+PD')
    wn = poly_or_wn;
%     poly = conv([1 wn*5], [1 sqrt(2)*wn wn^2]);
    poly = [1 2.1 3.4 2.7 1].*[1 wn wn^2 wn^3 wn^4];
elseif length(poly_or_wn) == 1
    wn = poly_or_wn;
    poly = poly_or_wn;
else
    poly = poly_or_wn;
end

order = length(poly) - 1;

% default = {poly};
% custom = [varargin default{(1 + length(varargin)):end}];
% [poly] = custom{:};

if strcmp(type, 'CMP+D')
    assert(order == 4 || order == 3, 'CMP+D expects a 3th or 4th order eq')
    if order == 4
        cp = (poly(2) - sum(gp));
        gc = tf(zpk([], -[gp; cp], 1));
        ds = gc.den{1};
        k2 = (poly(3) - ds(3))/G.k;
        k1 = (poly(4) - ds(4))/G.k - k2*cp;
        cz = (poly(5) - ds(5))/(G.k*k1);
        C1 = zpk(k1*(s + cz)/(s + cp));
        C2 = zpk(k2*s);
    elseif order == 3
        %assert, to escape a 3° order equation (that I'm lazy to compute)
        assert(ga(end) == 0, 'CMP+D accepts a 3th order eq only if G has an integrator');
        gaf = flip(ga);
        da = flip(poly - ga);
        cz = max(roots([1 (da(3)-gaf(3)) (gaf(2)-da(3)*gaf(3)+da(2))]));
        cp = da(3) + cz;
        k2 = (da(3)*cz - gaf(3)*da(3) + da(2))/G.k;
        k1 = da(1)/G.k;
        C1 = zpk(k1*(s + cz)/(s + cp));
        C2 = zpk(k2*s);
        assert(real(cz) > 0 && ~imag(cz), 'not possible! underdamp your dominant poles!');
    end
elseif strcmp(type, 'PI+PD')
    if order == 0
        gaf = flip(ga);
        zeta = 0.25*gaf(3)/wn;
        z1 = 3*wn; % cutoff at double the frequency
        k1 = wn^4/(z1*G.k);
        k2 = (4*zeta^2*wn^2 + 2*wn^2 - gaf(2))/G.k;
        z2 = (gaf(3)*wn^2 - G.k*k1)/(G.k*k2);
        C1 = zpk(k1*(s + z1)/s);
        C2 = zpk(k2*(s + z2));
    else
        assert(ga(end) == 0, 'PI+PD accepts a 3th order eq only if G has an integrator');
        assert(order == 3, 'PI+PD expects a 3th order poly')
        da = flip(poly - ga);
        gaf = flip(ga);
        k1 = da(1)/G.k;
        z1 = -da(3);
        k2 = (da(2) + (da(3)+gaf(3))*z1)/G.k;
        z2 = (da(2)+gaf(2))*z1/(k2*G.k);
        C1 = zpk(k1*(s + z1)/s);
        C2 = zpk(k2*(s + z2));
        assert(real(z1) > 0 && ~imag(z1), 'not possible! decrease the 2nd order coefficient');
    end
elseif strcmp(type, 'LPF+PDF')
    k = G.k;
    G = tf(G);
    gp = G.den{1}
    sp = (poly(2)-gp(2))
    pp = (poly(3)-gp(3)-gp(2)*sp)
    p1 = sp/2 - sqrt(sp^2-4*pp)/2
    p2 = sp/2 + sqrt(sp^2-4*pp)/2
    k2 = (poly(4)-gp(2)*pp-gp(3)*sp)
    k1 = -(poly(5)-gp(3)*pp-k2*sp)
    z2 = (poly(6)+k1*p2)/(k2*p1)
    C1 = zpk(k1/k/(s + p1))
    C2 = zpk(k2/k*(s + z2)/(s + p2))
end
end