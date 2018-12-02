clearvars -except id ctrl idn cid
clc

addpath(genpath('Gait Tracking With x-IMU'))
addpath('jsonlab-1.5')
addpath(genpath('CEE0099'))

%% GARBAGE
% 
pasta = 'main/';
foo = dir('main*');
realy = [dir([pasta '*']); foo];
for i = 3:length(realy)
    if i == length(realy)
        pasta = '';
    end
    if realy(i).isdir
        mat = dir([pasta realy(i).name '/' '*.fig']);
        for j = 1:length(mat)
            f = openfig([pasta realy(i).name '/' mat(j).name]);
            disp([pasta realy(i).name '/' mat(j).name])
            name = mat(j).name;
            name(end-3:end) = [];
            saveas(f, ['/run/media/pjago/FISHSTICK/TCC/tcc/figuras/' name], 'epsc')
            pause(1)
            close
        end
    end
end

%% Run (THIS SHOULD BE EXECUTED WITH CTRL + ENTER)

id0 = 2299; % first id here (there should be 12 in total)
ctrls = {'pitch', 'yaw', 'roll'};% 'height'};
if ~exist('id', 'var')
    for cid = 1:length(ctrls)
        for idn = 0:2
            id = num2str(id0 + (cid - 1)*3 + idn);
            ctrl = ctrls{cid};
            ctrl_plot;
            FOLDER = ['rold_sim/' ctrl '_sim/relay ' num2str(idn)];
            if ~exist(FOLDER, 'dir')
                mkdir(FOLDER);
            end   
            num = length(dir([FOLDER '/*.mat'])) + 1;
            save([FOLDER '/' num2str(num) '.mat'])
        end
    end
end

%%
% 1539009350(5352): correlação entre yaw e height

root = '/home/pjago/Desktop/Unity2/quad/Build/data/1542979784';
% root = '../../Unity2/quad/data/1542808648'; %'../data/1538404510';
id = '2677';
tag = 'pjago/quad/quad';
ctrl = 'ctrl/euler';
body = 'rigidbody';

% root = '/home/pjago/Desktop/Unity2/quad/Build/data/1542979784';
% % root = '../../Unity2/quad/data/1542848637';
% tag = 'pjago/quad/quad/ctrl';
% body = 'rigidbody';

D = load_csv([root '/' tag '/' ctrl '/' id '.csv']);
J = loadjson([root '/' tag '/' ctrl '/' id '.json']);
%P = loadjson([root '/' tag '/' 'aero' '/' id '.json']); %problem: no commas

%% CTRL

section = 1:J.n:height(D); %1:J.n:height(D);
t = (section-1)*J.dt;

% sat = @(u) sign(u).*min(max(abs(u), 0.1), 0.9);
sat = @(u) u;
if strcmp(ctrl, 'ctrl/props') || strcmp(ctrl, 'props') % this will look better with matrix mult
    u = [D.u D.u(:,1)-D.u(:,3) D.u(:,2)-D.u(:,4) sum(D.u(:,1:2)-D.u(:,3:4), 2) sum(D.u, 2)];
    d = [D.d D.d(:,1)-D.d(:,3) D.d(:,2)-D.d(:,4) sum(D.d(:,1:2)-D.d(:,3:4), 2) sum(D.d, 2)];
    e = [D.e D.e(:,1)-D.e(:,3) D.e(:,2)-D.e(:,4) sum(D.e(:,1:2)-D.e(:,3:4), 2) sum(D.e, 2)];
    r = [D.r D.r(:,1)-D.r(:,3) D.r(:,2)-D.r(:,4) sum(D.r(:,1:2)-D.r(:,3:4), 2) sum(D.r, 2)];
    y = [D.y D.y(:,1)-D.y(:,3) D.y(:,2)-D.y(:,4) sum(D.y(:,1:2)-D.y(:,3:4), 2) sum(D.y, 2)];
elseif strcmp(ctrl, 'ctrl/height') || strcmp(ctrl, 'height')
    u = 100*D.u;
    d = 100*D.d;
    e = D.e;
    r = D.r;
    y = D.y;
else
    D.y(D.y > 180) = D.y(D.y > 180) - 360;
    D.r(D.r > 180) = D.r(D.r > 180) - 360;
    u = 100*D.u;
    d = 100*D.d;
    e = D.e;
    r = D.r;
    y = D.y;
end

[hf, hfu] = plotudo(t(4449:(4449+250)), y, r, e, u, 0, 0, 1, 1, d);
hh = [];
for fidx = hf
    hfh = figure(fidx);
    hh = [hh hfh];
    yyaxis left
    if strcmp(ctrl, 'ctrl/height') || strcmp(ctrl, 'height')
        ylabel('Altura (m)')
    elseif strcmp(ctrl, 'ctrl/props') || strcmp(ctrl, 'props')
        ylabel('Torque (N.m)')
    else
        ylabel('Ângulo (°)')
    end
    yyaxis right
    if strcmp(ctrl, 'ctrl/props') || strcmp(ctrl, 'props')
       ylabel('Velocidade (rad/s)')
    else
       ylabel('Duty (%)')
    end
    gcf
end
% figure(hfu) 
% title(J.title)

% figure
% [R,Y] = meshgrid(-pi:0.1:pi,-pi:0.1:pi);
% Z = R - Y;
% C = R.*Y;
% h = surf(R,Y,Z);
% set(h, 'LineStyle', 'none')
% xlabel('R')
% ylabel('Y')
% zlabel('E')
% colorbar
% 
% figure
% [R,Y] = meshgrid(-pi:0.1:pi,-pi:0.1:pi);
% Z = sin(R) - sin(Y);
% h = surf(R,Y,Z);
% set(h, 'LineStyle', 'none')
% xlabel('R')
% ylabel('Y')
% zlabel('E')
% colorbar
% 
% figure
% [R,Y] = meshgrid(-pi:0.1:pi,-pi:0.1:pi);
% Z = sin(R - Y);
% h = surf(R,Y,Z);
% set(h, 'LineStyle', 'none')
% xlabel('R')
% ylabel('Y')
% zlabel('E')
% colorbar
% 
% figure
% [R,Y] = meshgrid(-pi:0.1:pi,-pi:0.1:pi);
% Z = (cos(R/2) - cos(Y/2));
% h = surf(R,Y,Z);
% set(h, 'LineStyle', 'none')
% xlabel('R')
% ylabel('Y')
% zlabel('E')
% colorbar


%% RIGIDBODY - ONLY FOR PLANE OR HEIGHT CTRL 

% Db = load_csv([root '/' tag '/' body '/' id '.csv']);
% Jb = loadjson([root '/' tag '/' body '/' id '.json']);
% section = 1:J.n:height(Db);
% t = (section-1)*J.dt;
% 
% euler = quatern2euler(Db.rotation); % it returns ZYX rotation, but Unity uses ZXY rotation
% yaw = euler(:, 2);
% % euler(euler >= 0) = euler(euler >= 0) - pi;
% % euler(euler < 0) = euler(euler < 0) + pi;
% euler(:, 2) = yaw;
% degree = euler*180/pi;
% 
% hf = figure;
% colorOrder = get(gca, 'ColorOrder');
% set(hf,'defaultAxesColorOrder',0.6*colorOrder);
% 
% yyaxis left
% plot(t, Db.position)
% ylabel('Posição (m)')
% xlabel('Tempo (s)')
% yyaxis right
% plot(t, euler*180/pi)
% ylabel('Ângulo (°)')
% xlabel('Tempo (s)')
% legend('x', 'y', 'z', 'pitch', 'yaw', 'roll')
% 
% %%
% rhpos = Db.position(1:end, [3 1 2]);
% % rotmat = quatern2rotMat(quatmultiply([cos(pi/4) 0 0 sin(pi/4)],...
% %                         Db.rotation(:, [1, 4, 2, 3])));
% rotmat = quatern2rotMat(Db.rotation(:, [1, 4, 2, 3]));
% 
% SixDofAnimation(rhpos, rotmat(:, :, section), 'Trail', 'All', 'ShowArrowHead', false, ...
%                 'SamplePlotFreq', 8, 'View', [130, 20],...
%                 'Xlabel', 'Z', 'Ylabel', 'X', 'Zlabel', 'Y');
%                         