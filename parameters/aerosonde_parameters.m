% initialize the mav viewer
addpath('../tools');  
clc;
clear all
close all
utils

% initial conditions
MAV.pn0    = 0;     % initial North position
MAV.pe0    = 30;     % initial Yg position
MAV.pd0    = 0;     % initial Zg position
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

% линейные размеры, мм (для графики)
MAV.radius_l = 125;    % луч от центра до оси винта
MAV.radius_x = MAV.radius_l*cos(pi/4);  % проекция луча на ось
MAV.radius_z = MAV.radius_l*cos(pi/4);
MAV.radius_a = 6;      % размер квадратного сечения луча
MAV.radius_a_x = MAV.radius_a*cos(45);
MAV.cockpit_side = 70;  % длина стороны кабины-куба
MAV.motor = [35 28];    % H, W параллелепипеда мотора

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
MAV.Motor.Umax = 4.2*4;         % 4S аккумулятор
MAV.Motor.Nmax = MAV.Motor.KV*MAV.Motor.Umax*(0.9)  % приблизительная максимальная скорость вращения

MAV.Motor

MAV.rho = 1.19;



% MAV.J = diag([MAV.Jx MAV.Jy MAV.Jz]);
% MAV.J = [ ...
%     1.34e-3 -9.4e-8 3.3e-8;
%     -9.4e-8 1.277e-3 5.2e-8;
%     3.3e-8 5.2e-8 2.383e-3
% ];
MAV.J = diag([1.34e-3 1.277e-3 2.383e-3]);
MAV.J_inv = inv(MAV.J);

filter_freq = 2*pi*1000;

% K_f = 5.2194e-10

Reg.PID1 = [
    200
    0
    10
]';
Reg.PID2 = [
    14
    0
    5
]';
Reg.PID3 = [
    85
    0
    60
]';
Reg.PID4 = [
    0
    0
    10
]';
Reg.PID5 = [
    0
    0
    5
]';
MAV.Reg = Reg;

% Modes
Modes = struct()

% Stabilize fast
Reg = struct()
Reg.PID1 = [
    200
    0
    10
]';
Reg.PID2 = [
    14
    0
    5
]';
Reg.PID3 = [
    85
    85
    25
]';
Modes.Stab_fast.Reg = Reg;
Modes.Stab_fast.N = 0;

% Flight on Beard targeting
Modes.Targeting_1.N = 1;
Reg = struct()
Reg.PID1 = [
    200
    0
    10
]';
Reg.PID2 = [
    0
    0
    5
]';
Reg.PID3 = [
    0
    0
    10
]';
Modes.Beard_targeting.Reg = Reg;
Modes.Beard_targeting.N = 2;

% Stabilize gently
Reg = struct()
Reg.PID1 = [
    200
    0
    10
]';
Reg.PID2 = [
    14
    0
    5
]';
Reg.PID3 = [
    85
    0
    60
]';
Reg.PID3_speed = [
    0
    0
    60
]';
Modes.Stab_gently.Reg = Reg;
Modes.Stab_gently.N = 3;

Modes.Soft_falling_1.Falling_speed = -0.3;

% Sleep; (7)
% Reg = struct();
% Reg.PID1 = zeros(3,1);
% Reg.PID2 = zeros(3,1);
% Reg.PID3 = zeros(3,1);
% Modes.Sleep.Reg = Reg;

MAV.Modes = Modes;

