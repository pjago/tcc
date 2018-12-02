function [ kc, ti, td ] = ziegler_nichols( Gjw, w, type )
%NICHOLS [ kp, ti, td ] = ziegler_nichols( Gjw, w, type )
%   PID parameters
tu = 2*pi/w;
ku = 1/abs(Gjw);
if strcmp(type, 'PID')
    kc = 0.6*ku;
    ti = 0.5*tu;
    td = 0.125*tu;
elseif strcmp(type, 'PD')
    kc = 0.8*ku;
    ti = Inf;
    td = tu/8;
elseif strcmp(type, 'PI')
    kc = 0.45*ku;
    ti = tu/1.2;
    td = 0.0;
elseif strcmp(type, 'P')
    kc = 0.5*ku;
    ti = Inf;
    td = 0.0;
elseif strcmp(type, 'PES')
    kc = 0.7*ku;
    ti = tu/2.5;
    td = 3*tu/20;
elseif strcmp(type, 'SO')
    kc = 0.33*ku;
    ti = tu/2;
    td = tu/3;
elseif strcmp(type, 'NO')
    kc = 0.2*ku;
    ti = tu/2;
    td = tu/3;
end
end