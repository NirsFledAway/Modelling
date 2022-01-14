function [a_y, telemetry, gamma_dot] = targeting_beard(r_t, dr_t, ddr_t, r_m, dr_m, ddm_t, theta, V_g, enable)
    if enable == 0
        gamma_dot = 0;
        a_y = 0;
        Omega = [0 0 0]'; a_y = 0; a_z = 0; l_dot = [0 0 0]'; l = [0 0 0]';
    else
    
        mu = 1;
        N = 10;
    
        l = r_t - r_m;
        l_dot = dr_t - dr_m;
        l_n = norm(l);
        
        R_g_v = Utils.getRotationMatrix([0 theta 0]);
        Omega = R_g_v * cross(l/l_n, l_dot/l_n); %нормированная(единичная) угловая скорость движения цели в СК коптера
    %Omega = cross(l/l_n, l_dot/l_n);
    
        a_y = mu*N*V_g*Omega(3);
%         a_z = -mu*N*V_g*Omega(2);
        a_z = 0;
        
        gamma_dot = sign(a_y)/V_g*sqrt(a_y^2+a_z^2);
       % gamma_dot = sign(a_z)/V_g*sqrt(a_x^2+a_y^2);
    end
    telemetry = [Omega, [0 a_y a_z]', l_dot, l];
%     telemetry = zeros(6, 1);