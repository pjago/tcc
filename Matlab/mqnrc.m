function Gs = mqnrc(y, u, T, varargin)
    %% Default values
    lpf_fc = 1/(50*T); %cutoff at 1/(50*T)
    lpf_tau = sqrt(sqrt(2) - 1)/(2*pi*lpf_fc); %2° order lpf
    input_lag = T; %about one sampling period of lag (+ ioDelay)
    default = {[1, 2], input_lag, lpf_tau, 1e-2, 1e-5}; %todo: weight Tol
    custom = [varargin default{(1 + length(varargin)):end}];
    [order, lag, alpha, AbsTol, RelTol] = custom{:};
    m = order(1); %numerator
    n = order(2); %denominator
    N = size(y, 1) - 1; %removing 1 here due the foh
    
    %% IDENTIFICATION OF CONTINUOUS-TIME SYSTEMS WITH UNKNOWN TIME DELAY BY NONLINEAR LEAST-SQUARES AND INSTRUMENTAL VARIABLE METHODS
    % Zi-Jiang Yang, Hideto Iemura, Shunshoku Kanae, and Kiyoshi Wada

    u_ = u((2:N+1));
    y_ = y(1:N);
    
%     eqr = @(i) deconv((T/2)^i*conv(convp([1 1], i), convp([1 -1], n-i)), ...
%                       convp(alpha*[1 -1] + T/2*[1 1], n)); %has remainder!!

% I am finding something with the pole signals changed, and the gain
% inverted, so I am missing a inversion somewhere

    eqr = @(i) mrdivide(LagOp((T/2)^i*conv(convp([1 1], i), convp([1 -1], n-i))), ...
        LagOp(convp(alpha*[1 -1] + T/2*[1 1], n)));
    E = arrayfun(eqr, 0:n, 'UniformOutput', false);
    drop = max(cellfun(@(x) x.Degree, E)); %lame
    
    at = @(x, i) subsref(x, struct('type', '()', 'subs', {{i}}));
    eps = @(i, f) at(qshift(f, E{i+1}, 1), (drop+1):N);
    phi = [];
    for i = 1:n
        phi = [phi -eps(i, y_)];
    end
    for i = n-m+1:n
        phi = [phi eps(i, u_)];
    end
    
    theta = (phi'*phi)\phi'*eps(0, y_);
    Gs = tf(theta((n+1):end)', [1 theta(1:n)']);
    Gs.inputDelay = 0;
end