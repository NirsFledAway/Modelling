% Подбираем коэффициенты ПИДов квадру по упрощенным ПФ
clc; close all; clear variables;

%%
aerosonde_parameters;
%% Pitch
Jz = MAV.J(3, 3)
rho = MAV.rho;
C_T = 0.0505;
% k_T = (1/Jz) * 1/6*rho*pi*(ed*d/2)^4*C_T * MAV.Prop.K 
k_T = 1.5230e-06 / Jz;

zeta = 0.7;
u_max = MAV.Control.u_xz_max * MAV.Prop.K_m/Jz
e_max = deg2rad(60);

k_p = u_max/e_max
w_n = sqrt(k_T*k_p);
k_d = 2*zeta*w_n/k_T
% k_d = 40000;

%%
close all;
W = tf([k_p*k_T], [1 k_d*k_T k_T*k_p]);
opt = stepDataOptions;
deg2rad(10)
opt.StepAmplitude = deg2rad(1);
step(W, opt)

%% calk thrust koeff
prop = MAV.Prop;
d = prop.d;
p = prop.p;
ed = prop.ed;
k = prop.Nb * prop.c_d / 2;
theta = atan(p/(pi*d));
%     lambda_c = Va./(Omega*d/2);
lambda_c = 0;

C_T = 4/3*k*theta*(1 - (1 - ed)^3) - ...
  k*( sqrt((lambda_c + k).^2+k) - sqrt(k) ) * (1-(1-ed)^2)

K_f = 1/6*rho*pi*(ed*d/2)^4*C_T;
K_f = K_f * prop.K