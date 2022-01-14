function [telemetry, theta] = targeting_forestalling(r_t, dr_t, ddr_t, r_m, dr_m, ddm_t, gamma, V_g, theta_euler, enable)
    if enable == 0
        theta = 0;
    else
        
    if -pi/2 <= gamma && gamma <= 0
        theta = gamma + deg2rad(20);
    else
        theta = gamma - deg2rad(20);
    end
    
    end
    
    telemetry = 0;
%     telemetry = zeros(6, 1);