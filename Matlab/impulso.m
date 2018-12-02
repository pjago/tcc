clc
%clear
format shortg
addpath(genpath('CEE0099'))

global t y r e u pwm k

%% CONFIG

% Adicione o nome de variaveis que queira salvar
toSave = {};
%toSave = {'t', 'y', 'r', 'e', 'u', 'pwm'};

T = 0.02;          %tempo de amostragem
n = 1001;          %numero de amostras
t = (0:(n-1))*T;  %vetor de tempo

%%



%% I/O

%caso nao ache a planta, o programa simula pela funcao de transferencia Gz
s = tf('s');
z = tf('z', T, 'variable', 'z^-1');
%utilizarei o roll identficado na simulação porque espera-se que exista um
%erro de ganho na prática, correspondente ao desequilíbrio entre os motores
%Gs = zpk(209.85/((s+1.535)*(s^2 + 2.345*s + 22.67))) % roll*
% Gs = zpk(104.92/((s+1.535)*(s^2 + 2.345*s))); % roll/2
Gs = zpk(23932 /(s*(s+3.044)*(s + 1.667)));
ps = Gs.p{1};
ps = ps(2:3); %these are the poles I want to reduce
pa = harmmean(ps)/2; % sum of their time constants
Ga = zpk(minreal(Gs*abs(pa)*tf(conv([1 -ps(1)],[1 -ps(2)]), [1 -pa]))/abs(prod(ps))); % roll/2
%Gs = zpk(93.035/((s+1.537)*(s^2 + 2.16*s + 22.36))); % roll_sim*
%Gs = zpk(93.035/((s+1.537)*(s^2 + 2.16*s))); % roll_sim
%Ga = zpk(25.165/(s*(s + 0.898))); % roll_sim
%Gs = zpk(92.939/((s+1.538)*(s^2 + 2.165*s + 22.35))); % pitch_sim*
%Gs = zpk(55.018/((s+1.158)*(s+2.833)*(s+0.06498))) % yaw_sim*
%Gs = zpk(55.018/((s+1.158)*(s^2+2.833*s))) % yaw_sim
%Ga = zpk(13.786/((s+0.822)*s)) % yaw_sim
%Gs = zpk(7.0815/((s+0.01183)*(s+0.3473)*(s+1.603))) % height_sim*
%Gs = zpk(7.0815/(s*(s+0.3473)*(s+1.603))) % height_sim
%Ga = zpk(3.6309/(s*(s+0.28545))) % height_sim

Gz = c2d(Gs, T);
Gz.variable = 'z^-1';

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom('COM20', Gz);

%% ESTADO INCIAL

[r, y, e, u, u1, u2, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

eps = 0.0;
u(2) = 0.01;
for k = 3:n
    %LEITURA
    time = tic;
    y(k) = read();

    %REFERENCIA E ERRO
    r(k) = 0.0;
%     r(k) = 1.0*(1-0.89284) + 0.89284*r(k-1);
%     r(k) = 1.4706;
    e(k) = r(k) - y(k);

    %CONTROLE

    % rltool LEAD+D (less efficient, low margin, fastest) @tcc
    % [C1, C2] = ctrl(Gs, 'CMP+D', [1 2.1 3.4 2.7 1].*[1 3.8 3.8^2 3.8^3 3.8^4])
    u1(k) = 0.11516*(e(k) - 0.6687*e(k-1)) + 0.9213*u1(k-1);
    u2(k) = 14.0991*(y(k) - y(k-1));

%     % rltool LEAD+D (more efficient, ~bad disturbance rejection) @tcc
%     % [C1, C2] = ctrl(Gs, 'CMP+D', [1 3 4 3 1].*[1 2.5 2.5^2 2.5^3 2.5^4])
%     u1(k) = 0.062691*(e(k) - 0.8596*e(k-1)) + 0.9231*u1(k-1);
%     u2(k) = 4.2553*(y(k) - y(k-1));
    
%     % a weird one (worst disturbance rejection, good margin) @tcc???
%     % [C1, C2] = ctrl(Gs, 'CMP+D', conv([1 2*5], [1 2*0.7*1.5 1.5^2]))
%     u1(k) = 0.21443*(e(k) - 0.973*e(k-1)) + 0.8236*u1(k-1);
%     u2(k) = -0.0208775*(y(k) - y(k-1));

%     % rltool LEAD+D (less efficient, low margin, minimum phase, fastest)
%     % [C1, C2] = ctrl(Gs, 'CMP+D', [1 2.1 3.4 2.7 1].*[1 5 5^2 5^3 5^4])
%     u1(k) = -0.5257*(e(k) - 1.212*e(k-1)) + 0.876*u1(k-1);
%     u2(k) = 26.5485*(y(k) - y(k-1));

    %I+PD by approximation @tcc
    %pa = harmmean(Gs.p{1}(2:3))/2;
    %rp = -0.8*pa; % time constant 25% bigger than approximation
    %[kp, ti, td] = pplace(Ga, conv([1 10*rp], [1 2*rp (rp/0.4)^2]), 'I+PD')
    %C = pid2(kp, kp/ti, kp*td, 0.0, 0.0, 0.0)
    %[C1, C2] = getComponents(C, 'feedback')
    u1(k) = 0.018907*r(k) - 0.018907*y(k) + u1(k-1);
    u2(k) = 15.021*(y(k)-0.9644*y(k-1));
    
%     %I+PD by approximation @tcc
%     %pa = harmmean(Gs.p{1}(2:3))/2;
%     %rp = -0.8*pa; % time constant 25% smaller than approximation
%     %[kp, ti, td] = pplace(Ga, conv([1 5*rp], [1 2*rp (rp/0.8)^2]), 'I+PD')
%     %C = pid2(kp, kp/ti, kp*td, 0.0, 0.0, 0.0)
%     %[C1, C2] = getComponents(C, 'feedback')
%     u1(k) = 0.0023634*e(k) + u1(k-1);
%     u2(k) = 8.0091*(y(k)-0.9706*y(k-1));

%     %rltool PI+PD (not @tcc)
%     %[kp, ti, td] = astron(-7.51, 0.70761*exp(-0.643j*pi), 1.9, 14.067)
%     %C = pid2(4.959*kp, kp/ti, kp*td, 0.0, 1/4.959, 0.0)
%     %[C1, C2] = getComponents(C, 'feedback')
%     u1(k) = 0.043178*(e(k) - 0.8972*e(k-1)) + u1(k-1); % matched
%     u2(k) = 5.3885*(y(k) - 0.9699*y(k-1)); % matched

%     %rltool PI+PD (not @tcc)
%     %[kp, ti, td] = astron(-7.51, 0.946*exp(-0.695j*pi), 1.9, 5.275)
%     %C = pid2(2.9*kp, kp/ti, kp*td, 0.0, 1/2.9, 0.0)
%     %[C1, C2] = getComponents(C, 'feedback')
%     u1(k) = 0.04092*(e(k) - 0.8916*e(k-1)) + u1(k-1);
%     u2(k) = 5.3913*(y(k) - 0.97*y(k-1)); % matched

%     %rltool PI+PD (not @tcc)
%     %
%     u1(k) = 0.026325*(e(k) - 0.9412*e(k-1)) + u1(k-1);
%     u2(k) = 3.056*(y(k) - 0.97*y(k-1));

%     %rltool PI+2Z1P (not @tcc)
%     %
%     u1(k) = 0.063657*(e(k) - 0.9798*e(k-1)) + u1(k-1);
%     u2(k) = 14.607*y(k) - 27.951*y(k-1) + 13.365*y(k-2) + 0.7649*u2(k-1);

    u(k) = u1(k) - u2(k);

    if e(k) > eps
        u(k) = abs(u(k-1));
    elseif e(k) < -eps
        u(k) = -abs(u(k-1));
    else
        u(k) = u(k-1);
    end
    
    %SATURACAO
    pwm(k) = u(k);
%     if u(k) > 100
%         pwm(k) = 100;
%     elseif u(k) < 0
%         pwm(k) = 0;
%     else
%         pwm(k) = u(k);
%     end
    
    %ESCRITA
    write(u(k));
    ping(k) = toc(time);
    
    %DELAY
    if isa(stop, 'function_handle')
        while toc(time) < T
        end
    end
end
stop();
fprintf('Duracao: %f seconds\n', toc(t0) - toc(time));
if sum(ping(1:end-1)' > T)
    disp('In-loop latency is too high! Increase your sampling time.')
end

fig = plotudo(t, 180/pi*y, r, e, 100*u, 0, 0, 1);

% subplot(2,1,1)
% axis([0 10 0 1.5])
% subplot(2,1,2)
% axis([0 10 -0.2 1.5])

if isa(stop, 'function_handle')
    folder = 'pratica';
else
    folder = 'teoria';
end
if ~exist(folder, 'dir')
    mkdir(folder);
end   
date = datestr(datetime('now'));
date(date == '-' | date == ':') = '_';
path = [folder '/' date];
disp(['Plant: ' folder ' NOT Saved at: ' path])