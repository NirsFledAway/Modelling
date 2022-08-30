function F = Gabriel_Staples(N, prop, rho, V0)
D_inch = prop.d;
D = D_inch*2.54/100;
H_inch = prop.h;
H = H_inch*2.54/100;
S = pi*D^2/4;

Ve = N*H/60;
% F_est = rho * S * (Ve.^2 - Ve*V0) * (D_inch/(3.29546*H_inch))^1.5;
F = rho * S * (Ve.^2 - Ve*V0) * (D_inch/(3.29546*H_inch))^1;
end