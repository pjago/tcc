clear
addpath(genpath('Gait Tracking With x-IMU'))
addpath('jsonlab-1.5')
addpath(genpath('CEE0099'))

reg0 = @(x) regexprep(x, '(\w)\((-?[0-9]+)\):', '\t$1: $2');
reg11 = @(x) regexprep(x, '[^\s]*:', '');
reg1 = @(x) x;
regl = @(x) regexprep(x, '(\w):[^\t]*', '\t$1');

%%
%%DON'T PRESS F5, DO ONE SECTION AT A TIME AND..
%%DO NOT RUN THIS SECTION IF ESC CONNECTION IS ON!!
%%TRUST ME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

COM = 'COM4';
s = serial(COM,'BaudRate',115200);
s.Timeout = 0.1;
s.terminator = 'LF';
s.RecordDetail = 'verbose';
s.RecordName = 'comlog.txt';
out = instrfind(s);
flushinput(s);
flushoutput(s);
if strcmp(out.status, 'closed')
    fclose(instrfind);
    fopen(s);
end
record(s, 'on');

head = @() repmat(reg0(fgets(s)), 2 , 1); % todo: parse (much simpler to do it in clojure)
read = @() subsref(textscan(reg1(fgets(s)),'%f'),struct('type','{}','subs',{{1}}));
write = @(x) fwrite(s, num2str(x));

%%

T = 0.02;
n = 1000;
h = nan(n, 1);
y = nan(n, 3);
u = nan(n, 4);
q = nan(n, 4);
debug = num2cell(repmat(' ', n, 1));

%for relay relay with histeresis: 'N', 'O', 'P', 'Q'
cmd = 'P';
fh = figure;
ph = plot(0, 0);
try
flushinput(s);
flushoutput(s);
flushinput(s);
write(cmd)
fgets(s)
for k = 1:n
    pause(0.00000001)
    debug{k} = fgets(s);
    debug{k}
    if ~isempty(debug{k})
        all = read();
        q(k, :) = all(1:4);
        y(k, :) = all(5:7);
        h(k) = all(8);
        u(k, :) = all(9:12);
    end
    ph.XData(1:k) = 1:k;
    ph.YData(1:k) = u(1:k, 2);
%     ph.YData(1:k) = y(1:k, 3);
%     ph.YData(1:k) = h(1:k);
end
catch ME
    disp(ME)
    disp('Try again!')
    if k <= 1
      close(fh)
    end
end

R = h; % temp
Y = y(:,2);
U = (u(:,1) - u(:, 3))/2;
% U = u/[-1 1 -1 1; -1 -1 1 1; 1 -1 -1 1; 1 1 1 1]'; % converts to [ux uy uz uh]

%%
figure
% ax1 = subplot(2, 1, 1);
tt = (0:n-2)*T;
hold on
plot(tt, y(1:n-1, 2), 'r')
plot(tt, y(1:n-1, 1), 'g')
plot(tt, y(1:n-1, 3), 'b')
legend('pitch', 'yaw', 'roll')
ylabel('Ã‚ngulo (Â°)')
xlabel('Tempo (s)')
grid minor
set(gca,'XMinorTick','on','YMinorTick','on')
set(gca,'MinorGridAlpha',0.5)

% ax2 = subplot(2, 1, 2);
% hold on
% plot(u(1:n, 1)/10, 'k')
% plot(u(1:n, 2)/10, 'Color', [0 0.447 0.741]);
% plot(u(1:n, 3)/10, 'Color', [0.9290 0.6940 0.1250]);
% plot(u(1:n, 4)/10, 'Color', [0.850 0.325 0.098]);
% legend('u0', 'u1', 'u2', 'u3')
% xlabel('amostra')
% ylabel('Duty (%)')
% 
% linkaxes([ax1 ax2], 'x')

%% Anim
Pos = [zeros(n, 2) h];
Rot = quatern2rotMat(q);
section = 1000:2000;
SixDofAnimation(Pos(section, :), Rot(:, :, section), 'Trail', 'All', 'ShowArrowHead', false, ...
                'SamplePlotFreq', 10, 'View', [130, 20]);


%%
close
g = 9.81;
arm = 0.325;
fmax = 641.283;
kSI = (pi/180)/(fmax*arm*g/1000);
% fx = @(u) ((sqrt(max(u, 10)*10) - b)/a).^2;
fx = @(u) u;
fs = @(p) min(max(p, 1100), 1300);
fu = @(t) (25.292*t.^0.5 + 6.331).^2 + 1000;
fuu = @(t) (21.570*(1.2445*t).^0.5 + 10.053).^2 + 1000;
ft = @(u) ((sqrt(u - 1000) - 6.331)/25.292).^2;
ku = (fmax*arm*g/1000); %10 is the operating point
ky = (pi/180);
Y = y(:, 3);
U = 100*(fx(u(:, 4)) - fx(u(:, 2)));
t = (0:n-1)*T;
[hf, ~] = plotudo(t(1:k), Y, h, (h-Y), 2*sign(U-mean(U)), 0, 0, -1);
figure(hf)
gcf
yyaxis right
set(findall(gca, 'Type', 'Line'),'LineWidth',0.01);
ylabel('Torque (%)')
ax = gca;
% ax.YAxis(2).Exponent = -3;
yyaxis left
ylabel('Ângulo (°)')

%% dropped frames extraction
op = cellfun(@(x) regl(reg0(x)), debug, 'UniformOutput', false);
code = cellfun(@(x) textscan(reg11(reg0(x)), '%.0f'), debug); %in reality is an int16
dopat = cellfun(@(x) ~isempty(strfind(x, 'd')), op);
dopsum = sum(dopat);
hasdop = [code{dopat}]';
dop = hasdop(:, 2)';

figure
hist(hasdop(:, 2)', 6); %not a normal distribution
title(sprintf('MPU6050 frames dropped: %.0f out of %.0f', dopsum, length(op)));
xlabel('bytes dropped')

%% timing
%discard first reading since first dt is not realiable (and we probably don't have a packet)
pst = cellfun(@(x) regexprep(x, '[c|d|e|n]\(-?[0-9]+\):', ''), {debug{1:end}}, 'UniformOutput', false);
pst = cell2mat(cellfun(@ (x) textscan(reg11(reg0(x)), '%.0f'), pst)');
pst = reshape(pst, [3 round(size(pst, 1)/3)])';
pst = array2table([pst(1:end, :) kmeans(pst(1:end, 3), 2)], 'VariableNames', {'p' 's' 't', 'group'});
pst_sorted = sortrows(pst, 'group');

%assert(sum(pst.p ~= 2) == dopsum, 'BAD FRAME!');
pdsc = fitdist(pst.s, 'normal');

figure
ax1 = subplot(2, 1, 1);
plot(pst.t, 'k', 'LineWidth', 0.01);
title('Período de Amostragem')
ylabel('us')
ax2 = subplot(2, 1, 2);
plot(pst.s, 'k', 'LineWidth', 0.01);
title('Atraso de processamento')
ylabel('us')
linkaxes([ax1 ax2], 'x')

figure
subplot(2, 1, 1)
hist(pst.t)
title('Período de Amostragem')
ylabel('N° de Amostras')
xlabel('us')
subplot(2, 1, 2)
histfit(pst.s)
title('Atraso de processamento')
ylabel('N° de Amostras')
xlabel('us')

%%
corrplot(array2table([pst.t pst.s], 'VariableNames', {'delta', 'delay'}))
title('Total Correlation')

pst_group = {};
last_group = 1;
t_group = [];
s_group = [];
for i = 1:height(pst)
    group = pst_sorted.group(i);
    t_group = [t_group; pst_sorted.t(i)];
    s_group = [s_group; pst_sorted.s(i)];
    if last_group ~= group || i == height(pst)
        pst_group{last_group} = array2table([t_group s_group], 'VariableNames', {'delta', 'delay'});
        corrplot(pst_group{last_group})
        title(sprintf('Cluster %.0f', last_group))
        t_group = [];
        s_group = [];
        last_group = group;
    end
end