function sys=mdlDerivatives(t,x,uu, MAV)
    
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
    R_g_b = Utils.getRotationMatrix([phi theta psi]);   % матрица поворота (g->b) {из Земной нормальной в связанную СК}

    cache.R_g_b = R_g_b;
    [Fb, Mb] = forces_moments(t,x,uu, MAV, cache);
    
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
function [Fb, Mb] = forces_moments(t,x,uu, MAV, cache)
    N = uu(1:4);    % скорость вращения двигов
    Omega = 2*pi*N/60;
    R_g_b = cache.R_g_b;
    
    % Forces
    f_gravity = R_g_b * [0; MAV.mass*(-MAV.gravity); 0];
    f = Gaurang(N, MAV.Prop, MAV.rho, x(5));
    f_thrust = [0 1 0]' * sum(f);
%     P_thrust = [0; 1; 0] * sum(f_thrust);   % тяго
    wind = R_g_b * MAV.Env.Wind_speed;
    f_aerial_drag = AerialDrag(x(4:6), wind, MAV.rho, MAV.Body.S, MAV.Body.C_aerial_drag_1);
    Fb = f_gravity + f_aerial_drag + f_thrust;    % результирующая сил в связанной СК

    % Moments
    
%     M_gyro = [mx my mz]';  % от винтов, вращение по рЫсканию
%     M_gyro = m;
    % отключено временно
%     M_y_aerial_vec = PropellerAeroMomentumPlain(N, MAV.Prop, MAV.rho); % за счет аэродинамического сопротивления
%     M_y_aerial = (-1) * sum(MAV.Prop.K_direction .*  M_y_aerial_vec);

    M_y_aerial = PropellerAeroMomentunParabole(N, MAV.Prop); % экспериментально подобранная зависимость, по факту включает в себя момент инерции
    
%     M_y_inertia = (MAV.Prop.J_y + MAV.Motor.J_rotor) .* sum(MAV.Prop.K_direction .* Omega.^2)
    M_y_inertia = 0;
%     M_y_aerial = 0;

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
    M = [1 -1 1 -1] * (Prop.C_aerial_momentum*(N + Prop.A_drift).^2);
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
function F = AerialDrag(v, w, rho, S, C)
    V = w + v;  % w - wind, v - vehicle speed;
    % S - характерная площадь
    F = - C * rho * V.^2/2 .* S;
end