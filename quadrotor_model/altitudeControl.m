function u = altitudeControl(x, MAV)
    %   Earth coordinates
%     X    = x(1);
    y    = x(2);
    z    = x(3);
%   speed in 'b' coordinates
    Vx     = x(4);
    Vy     = x(5);
    Vz     = x(6);
%   position quaternions
    phi    = x(7);
    theta    = x(8);
    psi    = x(9);
%   angular speeds
    Wx     = x(10);  % крен
    Wz     = x(11);  % тангаж
    Wy     = x(12);  % рыскание

% ---------

    K1 = [5000 100 1000]';
    persistent err_i
    if isempty(err_i)
        err_i = 0;
    end
    R_g_b = MAV.R_g_b([phi theta psi]);
    p_dot = R_g_b' * [Vx Vy Vz]';

    p_desired = [0 0 0]';

    err_p = y - p_desired(1);
    err_i = err_p + err_i;
    err_d = p_dot(2) - p_desired(2);

    err = [err_p err_i err_d]';

    % (a_T - a_c) + k*err = 0

    % u = MAV.mass*MAV.gravity - MAV.mass * (K1 .* err);
    u = sum(K1 .* err);
end