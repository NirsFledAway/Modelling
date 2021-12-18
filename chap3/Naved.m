function [phi_c, gamma_dot] = Naved(t,x,u,MAV,x_target_dot)
    
    mu = 1;
    N = 1;
    
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
    theta  = x(8);
    psi    = x(9);
%   angular speeds
    p     = x(10);  % крен
    q     = x(11);  % тангаж
    r     = x(12);  % рыскание
    x_target = x(13);

    R_g_b = MAV.R_g_b([phi theta psi]);

    l = [x_target 0 0]' - [pn pe pd]';
    p_dot = R_g_b' * [u v w]';
    l_dot = [x_target_dot 0 0]' - p_dot;
    l_n = norm(l);
    l_dot_n = norm(l_dot);
    R_g_v2 = MAV.R_g_b([0 theta psi]);
    Omega = R_g_v2 * cross(l_n, l_dot_n);

    
    V_g_ = (R_g_b') * [u v w]';
    V_g = norm(V_g_);
    V_g = u;

    a_y = mu*N*V_g*Omega(3);
    a_z = -mu*N*V_g*Omega(2);

    phi_c = atan(a_z/abs(a_y));
    gamma_dot = sign(a_y)/V_g*sqrt(a_y^2+a_z^2);
    