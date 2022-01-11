function [teta_c, u_1, u_2, err_p]=regul(state_, des_state_, k_pid, err_i)
state.p = state_(1:3);
state.v = state_(4:6);
state.euler = state_(7:9);
state.omega = state_(10:12);


des_state.p = des_state_(1:3);
des_state.v = des_state_(4:6);
des_state.acc = des_state_(7:9);

J_z = 2.383e-3;
K_f = Gaurang();
K_m = K_f*(125*cos(pi/4)*1e-3);
g = 9.81;
mass = 0.383;



err_p = [0; des_state.p(1)-state.p(1); des_state.p(2)-state.p(2)]; % ошибка по положению (вектор theta, х, у)
err_v = [0; des_state.v(1)-state.v(1); des_state.v(2)-state.v(2)]; % ошибка по скорости

k_theta = k_pid(1:3); %pid
k_x =     k_pid(4:6);
k_y =     k_pid(7:9);

a_x = des_state.acc(1) + k_x(3)*err_v(2) + k_x(1)*err_p(2);
teta_c = -a_x/(g);

err_p(1) = teta_c - state.euler(2);
err_v(1) = 0 - state.omega(2);

teta_ddot = 0 + k_theta(3)*err_v(1) + k_theta(1)*err_p(1) + k_theta(2)*err_i(1);
u_2 = teta_ddot*J_z/K_m;
Nmax = 40e3;
u_2_max = 2 * Nmax^2;
u_2 = min(u_2, u_2_max);

a_y = des_state.acc(2) + k_y(3)*err_v(3) + k_y(1)*err_p(3) + k_y(2)*err_i(3);
u_1 = (a_y + g)*mass/K_f;

u1_max = 4*Nmax^2 * (1-0.02);
u_1 = max(min(u_1, u1_max), 0);

function K_f = Gaurang()
    rho = 1.19;
    d = 0.127; % диаметр
    p = 0.1143;
    ed = 0.87;         % эффективность длины лопасти
    c_d = 0.1;         % отношение длины хорды к диаметру
    K_ = 4.8;             % поправочно-подгоночный коэффициент
    Nb = 3;            % число лопастей
    k = Nb * c_d / 2;
    theta = atan(p/(pi*d));
    lambda_c = 0;
    
    C_T = 4/3*k*theta*(1 - (1 - ed)^3) - ...
          k*( sqrt((lambda_c + k).^2+k) - sqrt(k) ) * (1-(1-ed)^2);
    
    K_f = 1/6*rho*pi*(ed*d/2)^4*C_T;
    K_f = K_f * K_;
    
    K_f = K_f * (2*pi/60)^2;