function T = Beard_UAV(N, prop, rho, Va, C_T)
    utils;
    
    A = pi*(inch2met(prop.d/2))^2;
%     C_T = 0.0061;
    
    V_i = N.*inch2met(prop.h)/60;
    
    T = 1/2*rho*A*C_T*(V_i.^2 - Va^2);
end