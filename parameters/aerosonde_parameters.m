% initialize the mav viewer
addpath('../tools');  
clc;
clear all
close all
utils

% initial conditions
MAV.pn0    = 0;     % initial North position
MAV.pe0    = 0;     % initial Yg position
MAV.pd0    = 0;     % initial Zg position
MAV.u0     = 0;     % initial velocity along body x-axis
MAV.v0     = 0;     % initial velocity along body y-axis

MAV.x0    = 0;     % initial North position
MAV.y0    = 0;     % для полета за целью
% MAV.y0    = 2;      % для стабилизации на точке
MAV.z0    = 0;     % initial Zg position
MAV.u0     = 0;     % initial velocity along body x-axis
MAV.v0     = 0;

MAV.pn0 = MAV.x0;
MAV.pe0 = MAV.y0;
MAV.pd0 = MAV.z0;

MAV.w0     = 0;     % initial velocity along body z-axis
MAV.phi0 = 0; % initial roll angle
MAV.psi0 = 0; % initial yaw angle
MAV.theta0 = 0; % initial pitch angle
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

MAV.Body.S = [5573 59852 6884] / (100*10)^2; % характерная площать квадра по осям в м^2
MAV.Body.C_aerial_drag_1 = 1.0; % 0.2 .. 1.2 Аэродинамическоий коэффициент сопротивления

MAV.rho = 1.19;

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
MAV.Prop.K_f = Gaurang(MAV.Prop, MAV.rho, 0);       % 1.6702e-08
MAV.Prop.K_m = MAV.Prop.K_f * (MAV.radius_z*1e-3);
MAV.Prop.C_aerial_momentum = 8.7300e-11;    % M = C*(Omega - a)^2 + b; см. Multirotor/RPM_detector/drag_momentum
MAV.Prop.A_drift = -4.8272e+03;
MAV.Prop.B_drift = -0.0023;

% Мотор
MAV.Motor.KV = 2400;
MAV.Motor.J_rotor = 6.8e-7;     % оценочный
MAV.Motor.Umax = 4.2*4;         % 4S аккумулятор
MAV.Motor.Nmax = MAV.Motor.KV*MAV.Motor.Umax*(0.9)  % приблизительная максимальная скорость вращения
MAV.Motor.T = 3.5/1000; % 0.007 по прикидкам статьи на медленный мотор, наш примерно в 1.5-2 раза рещще.

MAV.Motor;

MAV.Control.u_xz_max = 2 * (MAV.Motor.Nmax*0.9)^2;

% Environment
MAV.Env.Wind_speed = [
    0
    0
    0
];

% MAV.J = diag([MAV.Jx MAV.Jy MAV.Jz]);
% MAV.J = [ ...
%     1.34e-3 -9.4e-8 3.3e-8;
%     -9.4e-8 1.277e-3 5.2e-8;
%     3.3e-8 5.2e-8 2.383e-3
% ];
MAV.J = diag([1.34e-3 1.277e-3 2.383e-3]);
MAV.J_inv = inv(MAV.J);

filter_freq = 2*pi*1000;

%%% !!!Перенесено в get_pid_koeffs!!!
% % Modes
Modes = struct()
% 
% % Stabilize fast
% Reg = struct()
% Reg.PID1 = [
%     200
%     0
%     10
% ]';
% Reg.PID2 = [
%     14
%     0
%     5
% ]';
% Reg.PID3 = [
%     85
%     85
%     25
% ]';
% Modes.Stab_fast.Reg = Reg;
Modes.Stab_fast.N = 0;
% 
% % Flight on Beard targeting
Modes.Targeting_1.N = 1;
% Reg = struct()
% Reg.PID1 = [
%     200
%     0
%     10
% ]';
% Reg.PID2 = [
%     0
%     0
%     5
% ]';
% Reg.PID3 = [
%     0
%     0
%     10
% ]';
% Modes.Beard_targeting.Reg = Reg;
Modes.Beard_targeting.N = 2;
% 
% % Stabilize gently
% Reg = struct()
% % Reg.PID1 = [
% %     400
% %     0
% %     10
% % ]';
% % Reg.PID2 = [
% %     14
% %     0
% %     5
% % ]';
% Reg.PID1 = [
%     200
%     60
%     15
% ]';
% Reg.PID2 = [
%     12
%     6
%     7
% ]';
% % Reg.PID1 = [
% %     200
% %     0
% %     10
% % ]';
% % Reg.PID2 = [
% %     14
% %     0
% %     5
% % ]';
% Reg.PID3 = [
%     85
%     0
%     60
% ]';
% Reg.PID3_speed = [
%     0
%     0
%     60
% ]';
% Modes.Stab_gently.Reg = Reg;
Modes.Stab_gently.N = 3;
% 
Modes.Soft_falling_1.Falling_speed = -0.3;
% 
% % Sleep; (7)
% % Reg = struct();
% % Reg.PID1 = zeros(3,1);
% % Reg.PID2 = zeros(3,1);
% % Reg.PID3 = zeros(3,1);
% % Modes.Sleep.Reg = Reg;
% 
MAV.Modes = Modes;

targeting_method_select = 3;

% model_params
function K_f = Gaurang(prop, rho, Va)
    utils;
    d = prop.d;
    p = prop.p;
    ed = prop.ed;
    k = prop.Nb * prop.c_d / 2;
    theta = atan(p/(pi*d));
    %     lambda_c = Va./(Omega*d/2);
    lambda_c = 0;

    C_T = 4/3*k*theta*(1 - (1 - ed)^3) - ...
          k*( sqrt((lambda_c + k).^2+k) - sqrt(k) ) * (1-(1-ed)^2);

    K_f = 1/6*rho*pi*(ed*d/2)^4*C_T;
    K_f = K_f * prop.K;
    
    K_f = K_f * (2*pi/60)^2;
end
