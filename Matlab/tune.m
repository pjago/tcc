addpath(genpath('Gait Tracking With x-IMU'))
addpath('jsonlab-1.5')
addpath(genpath('CEE0099'))

tuning = @(Gs, args) pdfplace(Gs, args);
fOs = 0.8; %overshoot
ftp = 1; %peak time
fpsi = -log(fOs)/sqrt(pi^2+log(fOs)^2);
fwn = pi/(ftp*sqrt(1 - fpsi^2));
fpoles = fwn*[-fpsi+1j*sqrt(1-fpsi^2) -fpsi-1j*sqrt(1-fpsi^2)];
args = fpoles(1);

% A escolha de uma engine ao invés da função de transferência tem os seus
% prós e contras. Tomando isso em consideração chegou-se a aprox G2

% Gs = 209.85/((s+1.535)*(s^2 + 2.345*s + 22.67)) % roll
% Ĝs2 = 93.446/((s+1.533)*(s^2 + 2.15*s + 22.38)) % roll_sim (-1/2) (winner)
% Ĝs3 = 93.446/((s+1.426)*(s^2 + 2.356*s + 22.88)) % roll_sim (-1/2) (1.09*2.345)
% Ĝs1  = 98.336/((s+1.952)*(s^2 + 2.238s + 22.45)) % roll_sim (0)
% Ĝs0 = 92.282/((s+1.26)*(s^2 + 2.177*s + 22.93)) % roll_sim (-1)
% Ĝs = 93.054/((s+1.29)*(s^2 + 2.199*s + 22.84)) % pitch_sim (-1)
% Ĝs = 52.514/(s*(s+1.344)*(s+2.427)) % yaw_sim (-1)
% Ĝs = 7.2429/((s+0.02177)*(s+0.316)*(s+1.664)) % height_sim (-1)

%% RELAY INFO

% IMPORTANTE: Havia um bug na função relay_info, que calculava o eps um
% pouco menor do real. A função identificada foi um pouco diferente, mas os
% resultados são muito similares ao do TCC. A tabela Gjw tem os valores
% atualizados, porém o modelo e as figuras utilizaram os valores anteriores

% A imprecisão no ganho da função ao utilizar apenas o relé com histere é alta!
% uma possível maneira de reduzir a imprecisão é utilizar um relé com um integrador também!

% Talvez a melhor estratégia seria identificar o ganho do sistema G2, e não
% o ganho do sistema G1. Isso porque o primeiro pode ser calculado a partir
% da altura do nó, e do peso do drone. O segundo precisa de um teste ponto
% à ponto com os motores, ou seja, um teste adicional.

vbat = 11.1;
T = 0.02;
g = 9.81;
arm = 0.325;
wmax = 692.8944091372049;
fmax = 5.50960648204578;
dmax = 0.0897291946537407;
tmax = fmax*arm;
upu = 0.2;
a = 23.669; %23.994; %25.292
b = 6.331;
fu = @(t) (a*t.^0.5 + b).^2 + 1000;
fpu = @(p) ((sqrt(p*(2000 - 1000)) - b)/a).^2;
fxx = @(u) max(u, 10)/100;
ku = tmax;
ku_ = ku; % need to change cause I was dumb
ky = (pi/180);
kv = 104.71975511966; %*920/1000;
ke = 1/kv;
vmax = ke*wmax;
ra = 2*0.09;
eta = 0.75;
Ws = sqrt(upu)*wmax; % 259.918145048507; % 0.447213595499958*wmax ~ 0.2*tmax
Ms = 2*upu*dmax/Ws; % drag small ac gain
Ns = 2*upu*tmax/Ws; % thrust small ac gain
ktD = Ms/(2*Ws);
ktP = Ns/(2*Ws);
Km = 2*Ns*(ke/(ra*Ms/eta + ke^2)); % motors ddp to thrust gain (loop gain)
% Ke = (ke + Ms/(2*ke/ra*eta)); % turns ddp input to speed reference
esc = (dmax/(ke/ra)+vmax)/vbat; % percentage of nominal voltage employed
Ke = (vmax*(1/(2*sqrt(upu))-1/eta)+vbat*esc/eta)/wmax;
% Kw = Ke*Km*wmax; % turns speed reference to percent speed reference
% Ku = 2*Ws*Ke*Km*tmax; % turns speed reference to percent thrust
Ku = (ke/ra)/(Ms/eta+ke^2/ra)*(vmax/(2*sqrt(upu)) + dmax/(eta*(ke/ra)))*Ns;
Ku = tmax; %that's equals to tmax!!! amazing!
% Ku = (ke/ra)/(Ms/eta+ke^2/ra)*(vmax*(1/(2*sqrt(upu))-1/eta)+vbat*esc/eta)*Ns; % does not depends on eta!! nor upu

Gss = [];
G = [];
w = [];
rD = [];
rA = [];
rEPS = [];
% pasta = 'realz_sim/roll_sim/';
% pasta = 'realz_roll/';
pasta = 'rold/';
% pasta = 'rold_sim/roll_sim/';
realy = dir([pasta 'relay*']);
Gss = [];
for i = 1:length(realy)
    if realy(i).isdir
        mat = dir([pasta realy(i).name '/' '*.mat']);
        GG = [];
        WP = [];
        rDD = [];
        rAA = [];
        rEEPS = [];
        for j = 1:length(mat)
            load([pasta realy(i).name '/' mat(j).name])
            disp([pasta realy(i).name '/' mat(j).name])
            %pause(0.02)
            pause(1)
            close
            if strcmp(pasta, 'height_sim/')
                Y = D.y;
                U = D.u;
                ku = ku_/arm;
            elseif strcmp(pasta(end-4:end), '_sim/')
                Y = D.y*pi/180;
                U = D.u;
                ku = Ku;
%                 ku = tmax;
            elseif strcmp(pasta, 'roll/') || strcmp(pasta, 'realz_roll/') || strcmp(pasta, 'rold/')
                Y = (y(:,3) - h)*pi/180; % comb filter
                U = (u(:,4)-u(:,2));
                if strcmp(pasta, 'roll/')
                    U = U*(23.669/25.292)^2;
                end
%                 ku = tmax;
                ku = Ku;
            elseif strcmp(pasta, 'pitch/')
                Y = y(:, 2)*pi/180;
                U = (fpu(u(:, 1)/100*2) - fpu(u(:, 3)/100*2)); % saturou | não-linear
                ku = 2*ku_;
            end
            Gs = zpk(tf(NaN));
%             Gs = zpk(mqnrc(Y(500:end), U(500:end), T, [1, 3])); % desconsidere
            poles = Gs.p{1};
            for k = 1:length(poles)
                r = real(poles(k));
                if r > 0 && r < 1 && imag(poles(k)) == 0 % approximation error?
                    poles(k) = 0;
                    disp(['Moving real pole from ' num2str(r) ' to 0.' 10 ...
                        'Be sure your plant is stable!'])
                end
            end
            Gs.p{1} = poles;
            Gss = [Gss; Gs];
            [gg, wp, d, a, eps] = relay_info(Y(500:end), U(500:end), T);
            GG = [GG; gg];
            WP = [WP; wp];
            rDD = [rDD; d];
            rAA = [rAA; a];
            rEEPS = [rEEPS; eps];
        end
        G = [G GG];
        w = [w WP];
        rD = [rD rDD];
        rA = [rA rAA];
        rEPS = [rEPS rEEPS];
    end
end
Gss = zpk(Gss);
% pause(1)
% close all

%outliers
if strcmp(pasta, 'roll/')
%     Go = transpose(G(1,:));
%     wo = w(1,:)';
%     G(1,:) = NaN;
elseif strcmp(pasta, 'pitch/')
    Go = [G(1,2); G(3,1)];
    wo = [w(1,2); w(3,1)];
    G(3,1) = NaN;
    G(1,2) = NaN;
    w(3,1) = NaN;
    w(1,2) = NaN;
end

if  strcmp(pasta, 'roll/') || strcmp(pasta, 'rold/') || strcmp(pasta, 'realz_roll/')
%     w = mean(w, 1);
%     G = mean(G, 1);
end
if size(w, 1) > 1
    [wt, i] = sort(nanmean(w), 2);
    wt = wt';
    Gt = transpose(nanmean(G(:, i), 1));
    tD = transpose(nanmean(rD(:, i), 1));
    tA = transpose(nanmean(rA(:, i), 1));
    tEPS = transpose(nanmean(rEPS(:, i), 1));
else
    [wt, i] = sort(w, 2);
    wt = wt';
    Gt = transpose(G(:, i));
    tD = transpose(rD(:, i));
    tA = transpose(rA(:, i));
    tEPS = transpose(rEPS(:, i));
end

count = size(G);
count = count(1)*count(2);
ww = reshape(w, count, 1);
Gr = reshape(real(G), count, 1);
Gi = reshape(imag(G), count, 1);

Gu = [31.9208*exp(-2.3619j)
      27.7841*exp(-2.5390j)
      22.0009*exp(-2.7800j)
      18.6814*exp(-3.1410j)];
op = 1:size(Gt, 1);
Gp = Gt;
wp = wt;
Gp = Gt(op);
wp = wt(op);
Gp = Gp(op);
wp = wp(op);
s = tf('s');
wc = wt(end);
Kc = 1/abs(real(Gt(end)));

%% 1st order with integrador and RHPZ @tcc (shape)

kk = wc^2/Kc;
aa = 1/wc*sqrt(-1/(Kc*real(Gt(1)) + 1));
bb = aa*wc^2;
Gs = kk*(1 - aa*s)/(s*(s + bb));
Gfs = Gs/(1 + Gs);
[~, Pms] = margin(Gs);
[~, Pmfs] = margin(Gfs);

%% 2nd order with RHPZ

a = 0.23*100*ky/ku;
b = (wc + sqrt(wc^2 + 4*wc^2*(Kc^2 - Kc)))/(2*wc^2);
c = a*wc^2 - Kc;
d = sqrt(b/wc)/Kc;

Gs = (1 - d*s)/(a*s^2 + b*s + c);
Gfs = Gs/(1 + Gs);
[~, Pms] = margin(Gs);
[~, Pmfs] = margin(Gfs);

%% 2nd order @tcc (intuition)

SAB = 1./(abs(Gp).^2);
A = (real(Gp).*SAB).^2;
B = (imag(Gp).*SAB).^2;
dAB = diff(2*A + B);
dw2 = [diff(wp.^4) diff(wp.^2)];
x = dw2\dAB;

a = sqrt(x(1));
b = sqrt(B)./wp;
b(b < mean(b)) = []; %excluindo o crossover
b = mean(b);
c = (b^2 - x(2))/(4*a);
Ixx = a;
C = b;
K = c;
Gs = zpk((1/a)/(s^2 + b/a*s + c/a));%*exp(-s*0.02)
ks = 1/c;
wn = sqrt(c/a);
zeta = b/(2*wn);
Gfs = Gs/(1 + Gs);
[~, Pms] = margin(Gs);
[~, Pmfs] = margin(Gfs);
poles = Gs.p{1};
for i = 1:length(poles)
    r = real(poles(i));
    if r > 0 && r < 0.05 && imag(poles(i)) == 0 % approximation error?
        poles(i) = 0;
        disp(['Moving real pole from ' num2str(r) ' to 0.' 10 ...
            'Be sure your plant is stable!'])
    end
end
Gs.p{1} = poles;

%% 3rd order @tcc (chicken dinner)

SAB = 1./(abs(Gp(1:end)).^2);
B = (real(Gp(1:end)).*SAB).^2;
A = (imag(Gp(1:end)).*SAB).^2;

a = -diff(wp(1:end).^2)\diff(sqrt(A)./wp(1:end));
b = diff(wp(1:end).^2)\diff(sqrt(B));
% Kc = 0.135420404881499;
% wc = 4.377703679539747;
c = a*wc^2;
d = b*wc^2 - Kc;
Gs = zpk(1/(a*s^3 + b*s^2 + c*s + d));
Gfs = Gs/(1 + Gs);
poles = Gs.p{1};
[~, rank] = sort(real(poles), 'descend');
poles = poles(rank);
for i = 1:length(poles)
    r = real(poles(i));
    if r > 0 && r < 0.04 && imag(poles(i)) == 0 % approximation error?
        poles(i) = 0;
        disp(['Moving real pole from ' num2str(r) ' to 0.' 10 ...
            'Be sure your plant is stable!'])
    end
end
Gs.p{1} = poles;

G1 = tf(ku, [-1/poles(end) 1]);
if poles(1) ~= 0
    G2 = tf(1, conv([1 -poles(1)], [1 -poles(2)])/(dcgain(Gs)/ku*poles(1)*poles(2)));
    G2.den{1} = real(G2.den{1});
    den = G2.den{1};
else
    G2 = tf(-1, [1 -poles(2)]/(dcgain(Gs*s)/ku*poles(2)));
    G2.den{1} = real(G2.den{1});
    den = [G2.den{1} 0.0];
end

G2_ = G2;
G2_.den{1} = [G2.den{1}(1:2) 0.0];
Gs_ = zpk(G1*G2_);

Ixx = den(1);
C = den(2);
K = den(3);

Jp = Ms/-poles(end);
Jm = ke*ke/ra/-poles(end);
Jtp = (Ms+ke*ke/ra*eta)/-poles(end);

Ap = -Ke*Ke*eta/(Jtp*ra)-2*ktD/Jtp*Ws;
Bp = Ke*eta/(Jtp*ra);
Cp = ktD/Jtp*Ws^2;

[~, Pms] = margin(Gs);
[~, Pmfs] = margin(Gfs);

ps = Gs_.p{1};
ps = ps(2:3); %these are the poles I want to reduce
pa = harmmean(ps)/2; % sum of their time constants
Ga_ = zpk(minreal(Gs_*abs(pa)*tf(conv([1 -ps(1)],[1 -ps(2)]), [1 -pa]))/abs(prod(ps))); % roll/2

%% Control
% [kpp, tfp, tdp] = pdfplace(Gs, 0.99); %tuning(Gs, args);
% Cp = zpk(kpp*(1 + tdp*s/(1 + tfp*s)));

% I need to create the lshape version where I can choose a pole other than
% the integrator. TODO

[kpp, tip, tdp] = lshape(Gs, 5.6); % 3.4 for tilt, 3.8 for yaw, 3.8 for height
Cp = pid(kpp, kpp/tip, kpp*tdp);
Gf = minreal(Cp*Gs/(1 + Cp*Gs));
[~, Pmf] = margin(Gf);

%%

%zigler nichols
[kpn, tin, tdn] = ziegler_nichols(Gp(end), w(end), 'PD');
% [kpa, tia, tda] = astron(Gp(end), 1.5*exp(-0.75j*pi), wp(end), 0.25);
% tia = tia*4; % height identification is not right
% kpa = kpa*4; % sim results are not the expected
[kpa, tia, tda] = astron(Gp(2), 4.244*exp(-1j*pi/1.9404), wp(2), 1.0);
% [kpa, tia, tda] = astron(Gp(3), 2*exp(-j*pi/1.5), wp(2), 1.0);
Cn = pid(kpn, kpn/tin, kpn*tdn);
Ca = pid(kpa, kpa/tia, kpa*tda);

%%
f_0 = figure('rend','painters','pos',[275 183 640/1.745 321]);
h_0 = bodeplot(Gs);
bopts = bodeoptions;
setoptions(h_0,bopts);
hold on
margin(Gs)
children = get(f_0, 'Children');
granchildren = get(children(3), 'Children');
delete(granchildren(1));
granchildren = get(children(2), 'Children');
delete(granchildren(1));
axes(children(3))
xlim([1 10])
hold on
if min(size(w)) ~= 1
    plot(ww, 20*log(abs(Gr +1j*Gi))/log(10), 'b.')
    plot(wp, 20*log(abs(Gp))/log(10), 'rx')
    legend('G', 'medições', 'média', 'Location', 'Southwest')
else
    plot(ww, 20*log(abs(Gr +1j*Gi))/log(10), 'rx')
    legend('G simulado', 'medições', 'Location', 'Southwest')
end
ylim([-10 25])
xlim([1 10])
axes(children(2))
hold on
aa = 180/pi*angle(Gr +1j*Gi);
aa(aa > 0) = aa(aa > 0) - 360;
if min(size(w)) ~= 1
    plot(ww, aa, 'b.')
    aa = 180/pi*angle(Gp);
    aa(aa > 0) = aa(aa > 0) - 360;
    plot(wp, aa, 'rx')
    legend('G', 'medições', 'média', 'Location', 'Southwest')
else
    plot(ww, aa, 'rx')
    legend('G simulado', 'medições', 'Location', 'Southwest')
end
f_1 = figure('rend','painters','pos',[645 180 640/2 340]);
hold on
hn = nyquistplot(Gs);
nopts = nyquistoptions;
setoptions(hn,nopts);
setoptions(hn,'ShowFullContour', 'off');
if min(size(w)) ~= 1
    plot(Gr, Gi, 'b.')
    plot(real(Gp), imag(Gp), 'rx')
    legend('G', 'medições', 'média', 'Location', 'Southeast')
else
    plot(Gr, Gi, 'rx')
    legend('G simulado', 'medições', 'Location', 'Southeast')
end
% axis equal
axis([-10 10 -15 5])
% axis([-20 20 -35 5])
% axis equal
pbaspect([1 1 1])

%  ROLL (Servo)                                    11.1 V
%  5.0135             4.8709             4.6896
% -1.1538 - 0.0011i  -1.2356 - 0.2537i  -1.2454 - 0.5027i
%  54.125 / ((s+1.238)*(s^2 + 1.759s + 22.96))

%  ROLD (ServoTimer1)                              12.2 V
%  5.0974             4.8691             4.7716
% -1.3956 - 0.0122i  -1.5154 - 0.2424i  -1.6541 - 0.4977i
%  104.02 / ((s+1.71)*(s^2 + 2.578s + 21.58))

% realz_sim/roll_sim/relay 0/1.mat
%  d = 0.02, eps = 0.018229, eo = 5.376e-18, a = 0.26188,
%  w = 4.1337, Gjw = 10.2841 <-3.0719
% 
% realz_sim/roll_sim/relay 1/1.mat
%  d = 0.02, eps = 0.031843, eo = -1.696e-17, a = 0.28818,
%  w = 4.0277, Gjw = 11.3172 <-3.0309
% 
% realz_sim/roll_sim/relay 2/1.mat
%  d = 0.02, eps = 0.048923, eo = -4.3156e-19, a = 0.3194,
%  w = 3.927, Gjw = 12.5428 <-2.9878

% 4.169582519012661   4.004302186283064   3.854714912380114
% -12.090994321321894 - 2.171370084005867i

%%
f0 = figure;
hold on
nyquist(Gs)
nyquist(Ca*Gs)
nyquist(Cp*Gs)
legend('Gs', 'Ca*G', 'Cp*G', 'Location', 'Northwest')
axis equal
axis([-6 6 -10 10])

prn = @(x) num2str(x, '%.3f');
f00 = figure;
hold on
%step(0.071942446043165*Gs)
step(feedback(Ca*Gs, 1))
step(feedback(Cp*Gs, 1))

%%
f1 = figure;
hold on
margin(Gs)
margin(Gfs)
legend('malha aberta', 'malha fechada', 'Location', 'Southwest')
% ylim([-180 0])
xlim([1e-1 1e3])
title('Bode Diagram')

colorOrder = get(gca, 'ColorOrder');

yL=get(gca,'YLim');
xL=get(gca,'XLim');
txt1 = ['Pm = ' prn(Pms) '�'];
text((xL(1)+xL(2))/2-480,yL(2)-13,txt1,...
  'Color', colorOrder(1, :),...
  'HorizontalAlignment','left',...
  'VerticalAlignment','top',...
  'BackgroundColor',[1 1 1],...
  'FontSize',12);

yL=get(gca,'YLim');
xL=get(gca,'XLim');
txt1 = ['Pm = ' prn(Pmfs) '�'];
text((xL(1)+xL(2))/2-480,yL(2)-43,txt1,...
  'Color', colorOrder(2, :),...
  'HorizontalAlignment','left',...
  'VerticalAlignment','top',...
  'BackgroundColor',[1 1 1],...
  'FontSize',12);

f2 = figure;
hold on
margin(Gs)
margin(Gf)
legend('malha aberta', 'malha fechada', 'Location', 'Southwest')
% ylim([-180 0])
title('Bode Diagram')

yL=get(gca,'YLim');
xL=get(gca,'XLim');
txt1 = ['Pm = ' prn(Pms) '�'];
text((xL(1)+xL(2))/2-480,yL(2)-13,txt1,...
  'Color', colorOrder(1, :),...
  'HorizontalAlignment','left',...
  'VerticalAlignment','top',...
  'BackgroundColor',[1 1 1],...
  'FontSize',12);

yL=get(gca,'YLim');
xL=get(gca,'XLim');
txt1 = ['Pm = ' prn(Pmf) '�'];
text((xL(1)+xL(2))/2-480,yL(2)-43,txt1,...
  'Color', colorOrder(2, :),...
  'HorizontalAlignment','left',...
  'VerticalAlignment','top',...
  'BackgroundColor',[1 1 1],...
  'FontSize',12);

%%
f4 = figure;
rlocus(Cp*Gs)
xlim([-1/T 0])
ylim([-1/(2*T) 1/(2*T)])
grid on
axis equal
pbaspect([1 1 1])

f5 = figure;
hold on
bode(1/(1 + Gs))
bode(1/(1 + Ca*Gs))
bode(1/(1 + Cp*Gs))
legend('Gs', 'Ca*Gs', 'Cp*Gs', 'Location', 'Southwest')
title('Sensibilidade')