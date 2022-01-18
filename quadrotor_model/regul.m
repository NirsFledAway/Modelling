function [teta_c, u_1, u_2, err_p]=regul(state_, des_state_, err_i, x, mode, MAV)
% Unpack signals
state.p = state_(1:3);
state.v = state_(4:6);
state.euler = state_(7:9);
state.omega = state_(10:12);
state.acc = state_(13:15);

des_state.p = des_state_(1:3);
des_state.v = des_state_(4:6);
des_state.acc = des_state_(7:9);

% k_theta = k_pid(1:3); %pid
% k_x =     k_pid(4:6);
% k_y =     k_pid(7:9);
% 
flight_mode = struct();
if mode == 0
    flight_mode = MAV.Modes.Stab_fast;
elseif mode == 2
    flight_mode = MAV.Modes.Beard_targeting;
% elseif mode == 7
% %   sleep upon the treasures
%     flight_mode = MAV.Modes.Sleep;
else
    flight_mode = MAV.Modes.Beard_targeting;
%     MAV.Modes.Stab_gently = MAV.Modes.Stab_fast;
    flight_mode.Reg.PID1 = MAV.Modes.Stab_gently.Reg.PID1;
    if mode == 3 || mode == 5 || mode == 6
%     полет по Берду по ОХ
        flight_mode.Reg.PID2 = MAV.Modes.Stab_gently.Reg.PID2;
    end
    if mode == 4 || mode == 5
        flight_mode.Reg.PID3 = MAV.Modes.Stab_gently.Reg.PID3;
    end
    if mode == 6
%         flight_mode.Reg.PID3 = MAV.Modes.Stab_gently.Reg.PID3_speed;
    end
end

% if mode(1) == 1
%     k_x = MAV.Reg.PID5;
% end
% if mode(2) == 1
%     k_y = MAV.Reg.PID4;
% end

% mode
k_theta = flight_mode.Reg.PID1
k_x     = flight_mode.Reg.PID2
k_y     = flight_mode.Reg.PID3
% state

% Prepare coefficients
J_z = MAV.J(3, 3);
g = MAV.gravity;
K_f = Gaurang(MAV);
K_m = K_f * (MAV.radius_z*1e-3);
u_max = [
    4*(MAV.Motor.Nmax*0.5)^2; 
    2 * (MAV.Motor.Nmax*0.9)^2
];

% Calc control signal

err_p = [0; des_state.p(1)-state.p(1); des_state.p(2)-state.p(2)]; % ошибка по положению (вектор theta, х, у)
err_v = [0; des_state.v(1)-state.v(1); des_state.v(2)-state.v(2)]; % ошибка по скорости

a_x = des_state.acc(1) + k_x(3)*err_v(2) + k_x(1)*err_p(2) + k_x(2)*err_i(2);
teta_c = -a_x/(g);
teta_c_dot = - (k_x(1)*err_v(2)) / g;
% teta_c_dot = 0;
teta_c = sign(teta_c) * min(abs(teta_c), deg2rad(20));
% teta_c = 0;

err_p(1) = teta_c - state.euler(2);
err_v(1) = teta_c_dot - state.omega(2);

teta_ddot = 0 + k_theta(3)*err_v(1) + k_theta(1)*err_p(1) + k_theta(2)*err_i(1)
u_2 = teta_ddot*J_z/K_m;
u_2 = sign(u_2) * min(abs(u_2), u_max(2));
% u_2 = 0;

a_y = des_state.acc(2) + k_y(3)*err_v(3) + k_y(1)*cut_max(err_p(3), 3) + k_y(2)*err_i(3);
% err_v_y = err_v(3)
% err_p_y = err_p(3)
% err_i_y = err_i(3)
% aa_y = k_y(4)*(des_state.acc(2) - state.acc(2)) + k_y(3)*err_v(3) + k_y(1)*cut_max(err_p(3), 3) + k_y(2)*err_i(3) 
u_1 = (a_y + g)*MAV.mass/K_f;

u_1 = max(min(u_1, u_max(1)), 0);

% Считаем коэффициент силы (для линейной зависимости силы от N^2)
function K_f = Gaurang(MAV)
    prop = MAV.Prop;
    d = prop.d;
    p = prop.p;
    ed = prop.ed;
    k = prop.Nb * prop.c_d / 2;
    rho = MAV.rho;
    theta = atan(p/(pi*d));
    lambda_c = 0;
    
    C_T = 4/3*k*theta*(1 - (1 - ed)^3) - ...
          k*( sqrt((lambda_c + k).^2+k) - sqrt(k) ) * (1-(1-ed)^2);
    
    K_f = 1/6*rho*pi*(ed*d/2)^4*C_T;
    K_f = K_f * prop.K;
    
    K_f = K_f * (2*pi/60)^2;

function R = cut_max(val, max)
    R = sign(val) * min(abs(val), max);
