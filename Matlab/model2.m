function G2 = model2(Gn)
    %% MODEL2   G2 = model2(Gn)
    % Dado uma função de transferência contínua, retorna a função 
    % de segunda ordem que mais se assemelha.
    % TODO: escolher critério de aproximação, utilizar tipo de resposta
    % tempo de pico, overshoot / tempo de estabilização, overshoot / etc
    
    info = stepinfo(Gn);

    tp = info.PeakTime;
    Mu = info.Overshoot/100;
    
    zeta = -log(Mu)/sqrt(log(Mu)^2 + pi^2);
    wn = pi/(tp*sqrt(1 - zeta^2));
    
    p1 = -zeta*wn + 1j*sqrt(1 - zeta^2)*wn;
    p2 = -zeta*wn - 1j*sqrt(1 - zeta^2)*wn;

    G2 = zpk([], [p1 p2], dcgain(Gn)*wn^2);
end