function [Gjw, w, d, a, eps] = relay_info(y, u, T, varargin)
    %[Gjw, w] = relay_info(y, u, T)
    try %cleaning this once
    n = length(y);
    settled = (1:n > 0)'; % no shame
    u = u - mean(u(settled));
    spin = round(2*u/(peak2peak(u)));
%     spin = round(2*(u - mean(u))/(peak2peak(u)));
    e = mean(y(settled))-y;
    edg = settled.*[0; diff(spin)]/2;
    bet = logical(edg);
%     figure
%     hold on
%     plot(spin)
%     plot(edg)
    epsaux = e(bet).*(edg(bet));
    if ~isempty(epsaux) % todo: if find 1 cycle
        eo = median(e(settled));
        eps = max(0, median(epsaux));
    end
    % for measuring a, find the corresponding peak in beetween two edges!
    edg_id = sign(edg).*sign(spin.*settled).*(1:n)';
    edg_id(edg_id == 0) = [];
    edg_delta = [];
    aaux = [];
    if ~mod(length(edg_id), 2)
        edg_id(end) = [];
        edg(end) = [];
    end
    for ide = 1:length(edg_id)-1
        fst = edg_id(ide);
        lst = edg_id(ide+1)-1;
        edg_delta = [edg_delta; lst - fst + 1];
        aaux(ide, 1) = max(sign(edg(fst))*e(fst:lst));
        daux(ide, 1) = max(sign(edg(fst))*u(fst:lst));
    end
    if ~isempty(aaux) % todo: if find 1 cycle
        a = median(aaux);
    else
        a = NaN;
    end
    if ~isempty(daux) % todo: if find 1 cycle
        d = median(daux);
    else
        d = NaN;
    end
    if nargin >= 4
      d = varargin{1};
    end
    
    edg_delta = qshift(edg_delta, [1, 1]);
    w = 2*pi/(median(edg_delta)*T); 
    % could also be mode, if your relay is super precise
    
%     w = 2*pi*medfreq(u-mean(u), 1/T) %which one is less worst?
%     w = pi*maxfreq(u)/T;
    Gjw = -pi/(4*d)*sqrt(a^2 - eps^2) - 1j*pi/(4*d)*eps;
%     Gjw = -1/(2*d/(pi*a)*(sqrt(1 - ((eps + eo)/a)^2) + sqrt(1 - ((eps - eo)/a)^2)) - 1j*4*d*eps/(pi*a^2));
    %Lh = (asin(eps/a)/w); % this is what the delay was supposed to be
    %L = 2*T*finddelay(y, u) - Lh; % the rest is lag
    %L = T; % let's hard code here
    disp([' d = ' num2str(d) ', eps = ' num2str(eps) ', eo = ' num2str(eo) ', a = ' num2str(a) ',' 10 ' w = ' num2str(w) ', Gjw = ' num2str(abs(Gjw)) ' <' num2str(angle(Gjw)) 10])
    catch ME % todo: find the Black Swan
        eps = NaN;
        a = NaN;
        Gjw = NaN;
        w = NaN;
        disp(['Broke! ' ME.message])
    end
end