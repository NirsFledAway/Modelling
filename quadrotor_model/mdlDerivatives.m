function sys=mdlDerivatives(t, x, uu, MAV)
    
%   Earth coordinates
    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
%   speed in 'b' coordinates
    u     = x(4);
    v     = x(5);
    w     = x(6);
%   position quaternions
    phi    = x(7);
    theta    = x(8);
    psi    = x(9);
%   angular speeds
    p     = x(10);  % крен
    q     = x(11);  % рыскание
    r     = x(12);  % тангаж

    N = uu(1:4);
    
    wind = uu(5:7);
    R_g_b = Utils.getRotationMatrix([phi theta psi]);   % матрица поворота (g->b) {из Земной нормальной в связанную СК}

    cache.R_g_b = R_g_b;
    [Fb, Mb] = forces_moments(t,x,uu, wind, MAV, cache);
    
    % поступательная кинематика
    p_dot = R_g_b' * [u v w]';
    
    pn_dot = p_dot(1); pe_dot = p_dot(2); pd_dot = p_dot(3);    % x y z
    
    % вращательная кинематика
    tmp_matrix = [
        1, -cos(phi)*tan(theta), sin(phi)*tan(theta);
        0, cos(phi)/cos(theta),  -sin(phi)/cos(theta);
        0, sin(phi),             cos(phi)
    ];
    euler_angles = tmp_matrix * [p q r]';
    phi_dot = euler_angles(1); 
    psi_dot = euler_angles(2); theta_dot =  euler_angles(3);
    
    % поступательная динамика
    speed_dot = hat([u v w]')*[p q r]' + (1/MAV.mass) * Fb;
    u_dot = speed_dot(1); v_dot = speed_dot(2); w_dot = speed_dot(3);
    
    % вращательная динамика
    pqr_dot = MAV.J_inv * ( hat([p q r]) * MAV.J * [p q r]' + Mb );
    p_dot = pqr_dot(1); q_dot = pqr_dot(2); r_dot = pqr_dot(3);

    sys = [ ...
            pn_dot pe_dot pd_dot        ...
            u_dot v_dot w_dot           ...
            phi_dot theta_dot psi_dot   ...
            p_dot q_dot r_dot           ...
          ]';
end

% @return: Вектора силы и момента в связанной СК
function [Fb, Mb] = forces_moments(t,x,uu, wind_speed, MAV, cache)
    N = uu(1:4);    % скорость вращения двигов
    Omega = 2*pi*N/60;
    R_g_b = cache.R_g_b;
    
    % Forces
    f_gravity = R_g_b * [0; MAV.mass*(-MAV.gravity); 0];
    f = Gaurang(N, MAV.Prop, MAV.rho, x(5));
    f_thrust = [0 1 0]' * sum(f);
%     P_thrust = [0; 1; 0] * sum(f_thrust);   % тяго
%     wind = R_g_b * MAV.Env.Wind_speed;
    wind = wind_speed;
    f_aerial_drag = AerialDrag(x(4:6), wind, MAV.rho);
%     f_aerial_drag = [0 0 0]';
% f_aerial_drag = AerialDrag(x(4:6), wind, MAV.rho, MAV.Body.S, MAV.Body.C_aerial_drag_1);
    Fb = f_gravity + f_aerial_drag + f_thrust;    % результирующая сил в связанной СК

    % Moments
    
%     M_gyro = [mx my mz]';  % от винтов, вращение по рЫсканию
%     M_gyro = m;
    % отключено временно
%     M_y_aerial_vec = PropellerAeroMomentumPlain(N, MAV.Prop, MAV.rho); % за счет аэродинамического сопротивления
%     M_y_aerial = (-1) * sum(MAV.Prop.K_direction .*  M_y_aerial_vec);

    M_y_aerial = PropellerAeroMomentunParabole(N, MAV.Prop); % экспериментально подобранная зависимость, по факту включает в себя момент инерции
    
    % dOmega = 2*pi*dN/60;
%     M_y_inertia = (MAV.Prop.J_y + MAV.Motor.J_rotor) .* sum(MAV.Prop.K_direction .* dOmega.^2)
    M_y_inertia = 0;

    M_motors = [0 1 0]' * (M_y_aerial + M_y_inertia);

    M_gyro = [0 0 0]';

    M_traction = [
        ( f(1) + f(4) - (f(2) + f(3)) ) * MAV.radius_z*1e-3;
        0;
        ( f(1) + f(2) - (f(3) + f(4)) ) * MAV.radius_x*1e-3;
    ];

    

    Mb = M_gyro + M_traction + M_motors;
end

% Вычисление силы тяги по методу Gaurang
function T = Gaurang(N, prop, rho, Va)
    persistent K_f
    if isempty(K_f)
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
    end

    Omega = 2*pi*N/60;
    T = Omega.^2 .* K_f;
end

function M = PropellerAeroMomentunParabole(N, Prop)
%     Omega = 2*pi*N/60;
%     M = [1 -1 1 -1] * (Prop.C_aerial_momentum*(N + Prop.A_drift).^2);
    % для адекватной работы с регулятором без сдвига по аргументу
    M = [1 -1 1 -1] * (0.0000000001066078*(N.^2) + 0.0077448458147488); 
end

function M = PropellerAeroMomentumPlain(N, prop, rho)
    C = prop.C_1;
    S = prop.S_approx * sin(atan(prop.p/(pi*prop.d)));
    Omega = 2*pi*N/60;

    F = 1/2 * rho * S * C * (prop.d/2*Omega).^2;
    m_p = 3/4;      % точка приложения момента воздухом к винту
    M = F * (prop.d/2 * m_p);
end

% Сила аэродинамического сопротивления
% function F = AerialDrag(v, w, rho, S, C)
%     Va = v - w;  % w - wind, v - ground speed;
%     % S - характерная площадь
%     F = - C * rho * Va.^2/2 .* S';
%     
% end
function Fb = AerialDrag(Vb, Wb, rho)
    Va_vec = Vb - Wb;  % w - wind, v - ground speed; body frame
    Va = norm(Va_vec);
    alpha = atan2(Va_vec(2), Va_vec(1));    % attack angle
    beta = atan2(Va_vec(3), Va_vec(1));     % скольжения angle
    
    Va_0 = 10; % скорость, для которой рассчитаны зависимости сил
    rho_0 = 1.225;  % плотность воздуха при выведении зависимостей сил
%     C_0 = 0.715 / (0.5*rho_0*Va_0^2);  % if alpha = 0, beta = 0;
    % ----------------------------------------------------
    % alpha angle
    alpha = abs(alpha);
    alpha = sign(alpha)*mod(alpha, 180);
    alpha = alpha + 180 * sign(90-alpha)*(abs(alpha)>90);
    if alpha >= 0 && alpha <= 20
        F_xy = 0.6969 + 0.0022*(alpha+2.3409).^2;
    elseif alpha > 20 && alpha <= 90
        F_xy = -0.000435*alpha^2 + 0.098161*alpha - 0.007685;
%         C_d = 5.5300 - 4.3500e-04*(alpha - 112.8287)^2;
    elseif alpha >= -20 && alpha < 0
        F_xy = 0.6969 + 0.0022*(-alpha+2.3409).^2;
    else
        F_xy = 5.5300 - 4.3500e-04*(-alpha - 112.8287)^2;
    end
%     C_D_alpha = F_0 / (0.5*rho_0*Va_0^2) - C_0;
%     C_D_alpha = F_0 / (0.5*rho_0*Va_0^2);
        C_xy = F_xy / (0.5*rho_0*Va_0^2);
        C_x = C_xy * cos(alpha);
        C_y = C_xy * sin(alpha);
        
        
    
    %-----------------------------------------------------
    % beta angle
    b = abs(beta);
    b = sign(b)*mod(b, 180);
    b = b + 180 * sign(90-b)*(abs(b)>90);
    if b >= 0 && b <= 10
        F_xz = 2.81659e-7*b^6 - 5.42002e-6*b^5 + 0.0000194033*b^4 + 0.000149284*b^3 - 0.000766659*b^2 + 0.00100311*b + 0.715;
    elseif b > 10 && b <= 32
        F_xz = 0.000072941364*b^2+0.004725990899*b+0.676768850038;
    elseif b > 32 && b <= 47
        F_xz = 0.000014983590*b^3-0.002016505717*b^2+0.091338487517*b-0.446303186240;
    elseif b > 47 && b <= 66
        F_xz = -0.000005286789*b^3+0.000705178007*b^2-0.030877630605*b+1.390205322532;
    elseif b > 66 && b <= 90
        F_xz = 0.000000379469*b^3+0.000021666922*b^2-0.013615025702*b+1.599215019960;            
    else
        F_xz = 0;
    end
    C_xz = F_xz / (0.5*rho_0*Va_0^2);
    C_z = C_xz * sin(beta);
%     C_D_beta = F0 / (0.5*rho_0*Va_0^2) - C_0;
    
    
    % ----------------------------------------------
    % Drug force
%     F_D = 0.5*rho*Va^2 * (C_0 + alpha*C_D_alpha + beta*C_D_beta);
    
%     F_D = 0.5*rho*Va^2 * (C_D_alpha);
%     
%     Fa = [
%         (-1)*F_D;
%         0;
%         0;
%     ];  % в скоростной СК
%     c_a = cos(alpha); s_a = sin(alpha);
%     c_b = cos(beta); s_b = sin(beta);
%     % TODO: перепроверить, кто в кого переводитъ
%     R_a_b = [c_a*c_b -s_a*c_b s_b; s_a c_a 0; -c_a*s_b s_a*s_b c_b];    % из скоростной с связанную
% %     R_b_a = [ c_a s_a 0; -s_a c_a 0; 0 0 1];    % из связанной в скоростную
% %     Fb = R_b_a' * Fa;
%         Fb = R_a_b * Fa;
    Fb = (-1)*(1/2)*rho*Va^2 * [C_x; C_y; C_z];
end
    