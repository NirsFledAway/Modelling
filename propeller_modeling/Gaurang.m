function T = Gaurang(N, prop, rho, Va)
% utils;
Omega = 2*pi*N/60;
d = Utils.inch2met(prop.d);
p = Utils.inch2met(prop.h);
ed = prop.ed;
k = prop.Nb * prop.c_d / 2
theta = atan(p/(pi*d))
lambda_c = Va./(Omega*d/2)

C_T = 4/3*k*theta*(1 - (1 - ed)^3) - ...
      k*( sqrt((lambda_c - k).^2+k) - sqrt(k) ) * (1-(1-ed)^2)

% lambda_i = 1/2 * (sqrt((lambda_c-k).^2 + k) - (lambda_c+k));
% C_T = 2/3 * k * ed * (3*(lambda_i-2*theta)*ed + 2*theta*ed^2 + 6*theta - 6*lambda_i);

K_f = 1/6 * rho*pi*(ed*d/2)^4*C_T
% K_f = rho*pi*(d/2)^4*C_T

T = Omega.^2 .* K_f;