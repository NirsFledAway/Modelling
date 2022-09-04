clc; close all; clear variables;

% крен, тангаж
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

%% рыскание
e_max = deg2rad(5);    % максимальное отклонение по углу
Nmin = 0;
U_T_hover = 2249e5;
N_hover = sqrt(U_T_hover/4); % ~7499.3
% Условие: в режиме висения может поддерживать вертикальную тягу 
%(движки не достигают насыщения)
% N_hover = N_hover + 2000;
u_max = 2 * (N_hover + (N_hover - Nmin))^2 - 2*Nmin^2;  % 449800000
J_y = 1.277e-3;
rho = 1.19;
C_m = 1.066078e-10; % коэффициент сопротивления пропеллера
k_T = C_m / J_y;
 
ksi = 0.9;    % колебательность контура, задаётся

%%
k_p = u_max / e_max
w_n = sqrt(k_T*k_p);    % собственная частота колебательного звена
k_d = 2*ksi*w_n / k_T
W_theta = tf([k_p*k_T], [1 k_d*k_T k_T*k_p])
%%
bode(W_theta);
%%
opt = stepDataOptions;
opt.StepAmplitude = deg2rad(5);
close all;
figure;
step(W_theta, opt)

%%