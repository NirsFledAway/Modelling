function [u_1, u_2, teta_c]=regul(data)
global MAV;
state = data(1);
des_state = data(2);
k_pid = data (3)

J_z = MAV.J(3, 3);
K_f = MAV.temp.K_f;
K_m = MAV.temp.K_m;

err_p = [0; des_state.p(1)-state.p(1); des_state.p(2)-state.p(2)] % ошибка по положению (вектор theta, х, у)
err_v = [0; des_state.v(1)-state.v(1); des_state.v(2)-state.v(2)] % ошибка по скорости

k_theta = k_pid(1:3); %pid
k_x =     k_pid(4:6);
k_y =     k_pid(7:9);

a_x = des_state.acc(1) + k_x(3)*err_v(2) + k_x(1)*err_p(2);
teta_c = -a_x/(K_f*g);

err_p(1) = teta_c - state.euler(2);
err_v(1) = 0 - state.omega(2);

teta_ddot = 0 + k_theta(3)*err_v(1) + k_theta(1)*err_p(1);
u_2 = teta_ddot*J_z/K_m;

a_y = des_state.acc(2) + g + k_y(3)*err_v(3) + k_y(1)*err_p(3);
u_1 = (a_y + g)*MAV.mass/K_f;