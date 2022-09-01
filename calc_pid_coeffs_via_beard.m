clc; close all; clear variables;

e_max = deg2rad(60);    % максимальное отклонение по углу
Nmax = 36288;
u_max = 2*Nmax^2 % 2.6336e+09 - показатель максимально возможного момента
J_z = 2.383e-3;
J_x = 1.34e-3;
rho = 1.19;
K_m = 1.4763e-09;       % коэффициент тяги пропеллера
% k_T = K_m/J_z;    % from Guarang thrust
k_T = K_m / J_x;
 
ksi = 0.9;    % колебательность контура, задаётся
% тангаж
% 0.8 - 0.095sec 1.5% overshoot
% 0.9 - 0.12sec
% 1.0 - 0.148sec

% крен

%%
k_p = u_max / e_max
w_n = sqrt(k_T*k_p);    % собственная частота колебательного звена
k_d = 2*ksi*w_n / k_T

%%
W_theta = tf([k_p*k_T], [1 k_d*k_T k_T*k_p])
bode(W_theta);
%%
opt = stepDataOptions;
opt.StepAmplitude = deg2rad(20);
step(W_theta, opt)

%%