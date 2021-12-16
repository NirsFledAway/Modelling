function T = Gaurang(N, prop, rho, V0)

Omega = 2*pi*N/60;
inch2met = @(val) val*2.54/100;
d = inch2met(prop.d);
p = inch2met(prop.h);
ed = prop.ed;
k = prop.Nb * prop.c_d / 2

theta = atan(p/(pi*d))

C_T = 4/3*k*theta*(1 - (1 - ed)^3) - ...
      k*( sqrt(k*(1+k)) - sqrt(k) ) * (1-(1-ed)^2)

% K_f = 1/6*rho*pi*(d/2)^4*ed^4*C_T
K_f = rho*pi*(d/2)^4*C_T

T = Omega * K_f;
T