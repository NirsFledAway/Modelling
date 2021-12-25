% initialize the mav viewer
addpath('../tools');  
clc;
clear all
close all
utils

% initial conditions
MAV.pn0    = 0;     % initial North position
MAV.pe0    = 0;     % initial East position
MAV.pd0    = 0;  % initial Down position (negative altitude)
MAV.u0     = 0;     % initial velocity along body x-axis
MAV.v0     = 0;     % initial velocity along body y-axis
MAV.w0     = 0;     % initial velocity along body z-axis
MAV.phi0 = 0; % initial roll angle
MAV.psi0 = 0; % initial yaw angle
MAV.theta0 =  0; % initial pitch angle
e = Euler2Quaternion(MAV.phi0, MAV.theta0, MAV.psi0);
MAV.e0     = e(1);  % initial quaternion
MAV.e1     = e(2);
MAV.e2     = e(3);
MAV.e3     = e(4);
MAV.p0     = 0;     % initial body frame roll rate
MAV.q0     = 0;     % initial body frame pitch rate
MAV.r0     = 0;     % initial body frame yaw rate
   
%physical parameters of airframe
MAV.gravity = 9.81;
MAV.mass = 0.383;
% // MAV.Jx   = 0.824;
% // MAV.Jy   = 1.135;
% // MAV.Jz   = 1.759;
% // MAV.Jxz  = 0.120;
% линейные размеры, мм (для графики)
MAV.radius_l = 125;    % луч от центра до оси винта
MAV.radius_x = MAV.radius_l*cos(pi/4);  % проекция луча на ось
MAV.radius_z = MAV.radius_l*cos(pi/4);
MAV.radius_a = 6;      % размер квадратного сечения луча
MAV.radius_a_x = MAV.radius_a*cos(45);
MAV.cockpit_side = 70;  % длина стороны кабины-куба
MAV.motor = [35 28];    % H, W параллелепипеда мотора


% // MAV.S_wing        = 0.55;
% // MAV.b             = 2.90;
% // MAV.c             = 0.19;
% // MAV.S_prop        = 0.2027;
% // MAV.rho           = 1.2682;
% // MAV.e             = 0.9;
% // MAV.AR            = MAV.b^2/MAV.S_wing;

% Пропеллер
MAV.Prop.d = inch2met(5.1); % диаметр
MAV.Prop.p = inch2met(4.5);   % шаг
MAV.Prop.ed = 0.87;         % эффективность длины лопасти
MAV.Prop.c_d = 0.1;         % отношение длины хорды к диаметру
MAV.Prop.K_direction = [1 -1 1 -1]';
MAV.Prop.K = 4.8;             % поправочно-подгоночный коэффициент
MAV.Prop.Nb = 3;            % число лопастей
MAV.Prop.S_approx = 5.5e-4; % m^2
MAV.Prop.C_1 = 1.15;        % 1.15 .. 1.25 according to (p. 72): https://cyberleninka.ru/article/n/aerodinamicheskoe-soprotivlenie-ploho-obtekaemyh-tel/viewer
MAV.Prop.J_y = 0;

% Мотор
MAV.Motor.KV = 2400;
MAV.Motor.J_rotor = 6.8e-7;     % оценочный

MAV.rho = 1.19;



% MAV.J = diag([MAV.Jx MAV.Jy MAV.Jz]);
% MAV.J = [ ...
%     1.34e-3 -9.4e-8 3.3e-8;
%     -9.4e-8 1.277e-3 5.2e-8;
%     3.3e-8 5.2e-8 2.383e-3
% ];
MAV.J = diag([1.34e-3 1.277e-3 2.383e-3]);
MAV.J
MAV.J_inv = inv(MAV.J);
MAV.R_g_b = @getRotationMatrix;

MAV.Cache.V_dot = [0 0 0]';

% @angles = [phi theta psi]
function R = getRotationMatrix(angles)
    phi = 1; t = 2; psi = 3; %indices
    s = sin(angles);
    c = cos(angles);
    R = [...
        c(psi)*c(t), s(phi)*s(psi) + c(phi)*c(psi)*s(t), c(psi)*s(phi)*s(t) - c(phi)*s(psi); ...
        -s(t),      c(phi)*c(t),                  c(t)*s(phi);                     ...
        c(t)*s(psi), c(phi)*s(psi)*s(t) - c(psi)*s(phi),    c(phi)*c(psi) + s(phi)*s(psi)*s(t)   ...
    ];
end
