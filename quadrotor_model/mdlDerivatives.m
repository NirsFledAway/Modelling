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
    q     = x(11);  % тангаж
    r     = x(12);  % рыскание

    % N = uu(1:4);
%   inputs
%     Ncontrol = altitudeControl(x, MAV);
%     uu(1:4) = [Ncontrol Ncontrol Ncontrol Ncontrol]';
    N = uu(1:4);

%     p = x(1:3)
%     V = x(4:6);
%     euler = x(7:9);
%     omega = x(10:12);
%     f = uu(1:3)
%     m = uu(4:6);

    [Fb, Mb] = forces_moments(t,x,uu, MAV);
    
    R_g_b = MAV.R_g_b1([phi theta psi]); % матрица поворота (g->b) {из Земной нормальной в связанную СК}
    
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

    MAV.Cache.V_dot = [u_dot v_dot w_dot]'; % линейные ускорения
end

% @return: Вектора силы и момента в связанной СК
function [Fb, Mb] = forces_moments(t,x,uu, MAV)
    euler_angles = x(7:9);
    N = uu(1:4);    % скорость вращения двигов
    Omega = 2*pi*N/60;
%     m = uu(5:7);
    %   angular speeds
    w_x     = x(10);  % крен
    w_y     = x(11);  % тангаж
    w_z     = x(12);  % рыскание


    R_g_b = MAV.R_g_b1(euler_angles); % матрица поворота (g->b) {из Земной нормальной в связанную СК}

    % Forces
    f_gravity = R_g_b * [0; MAV.mass*(-MAV.gravity); 0];
    f_resistance = [0; 0; 0];
    f = Gaurang(N, MAV.Prop, MAV.rho, x(5));
    f_thrust = [0 1 0]' * sum(f);
%     P_thrust = [0; 1; 0] * sum(f_thrust);   % тяго
    Fb = f_gravity + f_resistance + f_thrust;    % результирующая сил в связанной СК

    % Moments
%     M_gyro = [mx my mz]';  % от винтов, вращение по рЫсканию
%     M_gyro = m;
    M_y_aerial_vec = PropellerAeroMomentumPlain(N, MAV.Prop, MAV.rho); % за счет аэродинамического сопротивления
    M_y_aerial = (-1) * sum(MAV.Prop.K_direction .*  M_y_aerial_vec);

    M_y_aerial = 0;
    
    M_y_inertia = (MAV.Prop.J_y + MAV.Motor.J_rotor) .* sum(MAV.Prop.K_direction .* Omega.^2);

    M_motors = [0 1 0]' * (M_y_aerial + M_y_inertia);
%     M_motors = [0 0 0]';

%     omega_motor_sum = sum([1 -1 -1 1]' .* Omega);

    M_gyro = [0 0 0]';
%     M_gyro = (MAV.Prop.J_y + MAV.Motor.J_rotor) * w_z * omega_motor_sum;

    M_traction = [
        ( f(1) + f(4) - (f(2) + f(3)) ) * MAV.radius_z*1e-3;
        0;
        ( f(1) + f(2) - (f(3) + f(4)) ) * MAV.radius_x*1e-3;
    ];
    Mb = M_gyro + M_traction + M_motors;
end

% Вычисление силы тяги по методу Gaurang
function T = Gaurang(N, prop, rho, Va)
    utils;
    Omega = 2*pi*N/60;
    d = prop.d;
    p = prop.p;
    ed = prop.ed;
    k = prop.Nb * prop.c_d / 2;
    theta = atan(p/(pi*d));
    lambda_c = Va./(Omega*d/2);
    lambda_c = 0;
    
    C_T = 4/3*k*theta*(1 - (1 - ed)^3) - ...
          k*( sqrt((lambda_c + k).^2+k) - sqrt(k) ) * (1-(1-ed)^2);
    
    K_f = 1/6*rho*pi*(ed*d/2)^4*C_T;
    K_f = K_f * prop.K;
    
    T = Omega.^2 .* K_f;
end

function M = PropellerAeroMomentumPlain(N, prop, rho)
    C = prop.C_1;
    S = prop.S_approx * sin(atan(prop.p/(pi*prop.d)));
    Omega = 2*pi*N/60;

    F = 1/2 * rho * S * C * (prop.d/2*Omega).^2;
    m_p = 3/4;      % точка приложения момента воздухом к винту
    M = F * (prop.d/2 * m_p);
end