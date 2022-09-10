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
opt = stepDataOptions;
opt.StepAmplitude = deg2rad(5);
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
opt = stepDataOptions;
opt.StepAmplitude = deg2rad(5);
%%
k_p = u_max / e_max
w_n = sqrt(k_T*k_p)    % собственная частота колебательного звена
k_d = 2*ksi*w_n / k_T
W = tf([k_p*k_T], [1 k_d*k_T k_T*k_p])
%% Х
m = 0.383;
g = 9.81;
T = m * g;
e_max = 1.5;
u_max = deg2rad(45);
A = 1/g;

zeta = 1;
k_p = u_max / e_max
% k_p = k_p * 2
w_n = sqrt(k_p/A);    % собственная частота колебательного звена
k_d = 2*zeta*w_n*A
% k_i = (T*k_p)/(2*zeta)^2 * (1/T)
k_i = 0;



W_pd = tf([k_p/A], [1 k_d/A k_p/A])
W_pid = tf([k_p/A k_i/A], [1 k_d/A k_p/A k_i/A])
W_open = tf([g], [1 0 0]);

W = W_pid

opt = stepDataOptions;
opt.StepAmplitude = 1;    % meters
%% Vx малая
g = 9.81;
A = 1/g;

zeta = 0.7;
w_n_theta = 39.4720;    % rad/s
w_n_phi = 52.6379;
% w_n_x = (1/5) * w_n_theta
w_n_x = (1/8) * w_n_phi
k_i = w_n_x^2 * A
k_p = 2*zeta*w_n_x * A

W_pd = tf([k_p/A k_i/A], [1 k_p/A k_i/A])
W_open = tf([1/g], [1 0]);

W = W_pd

opt = stepDataOptions;
opt.StepAmplitude = 10;    % meters

close all;
figure;
step(W, opt)
%%
bode(W)
%% old
k_p = 50;
k_d = 10;
k_i = 50;
%% new с мелким интегратором
k_p = 3.9
k_d = 1.1339
k_i = 0.1
%% хз откуда но робит норм
k_p = 5.7721
k_d = 1.6397
k_i = 0.1;
%% new с интегратором крупнее и ограничением на него 0.1
k_p = 4.8313;
k_i = 0.5;
k_d = 0.9931;

%%
syms psii
getRotationMatrix([0 0 psii])
function R = getRotationMatrix(angles)
            phi = 1; t = 2; psi = 3; %indices
            s = sin(angles);
            c = cos(angles);
%             R = [...
%                 c(psi)*c(t), s(phi)*s(psi) + c(phi)*c(psi)*s(t), c(psi)*s(phi)*s(t) - c(phi)*s(psi); ...
%                 -s(t),      c(phi)*c(t),                  c(t)*s(phi);                     ...
%                 c(t)*s(psi), c(phi)*s(psi)*s(t) - c(psi)*s(phi),    c(phi)*c(psi) + s(phi)*s(psi)*s(t)   ...
%             ];
            R = [
                c(t)*c(psi) s(t)    -c(t)*s(psi);
                -c(phi)*s(t)*c(psi)+s(phi)*s(psi) c(phi)*c(t) c(phi)*s(t)*s(psi)+s(phi)*c(psi);
                s(phi)*s(t)*c(psi)+c(phi)*s(psi)  -s(phi)*c(t)    -s(psi)*s(t)*s(phi)+c(psi)*c(phi);
            ];
        end