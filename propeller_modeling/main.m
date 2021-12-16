clc; clear variables; close all;
data;
% n = [15700 19700 23420 40320]; % RPM
% F = [2.943 4.905 6.867 12.6549]; % Thrust, Newtons
% F = [0.74556 1.79523 2.77623 3.45312 4.17906 4.88538 5.4936 6.17049 6.78852 7.39674 7.96572 8.61318 9.18216 9.78057 10.04544];
% n = [7220 10790 13030 14720 16180 17150 18460 19270 20270 21060 21840 22590 23210 23920 24560];
% D_inch = 5;
% D = D_inch*2.54/100;
% H_inch = 4;
% H = H_inch*2.54/100;
rho = 1.2041;
% % rho = 1.225;
% S = pi*D^2/4;
% 
% Ve = n*H/60;
V0 = 0;
% % F_est = rho * S * (Ve.^2 - Ve*V0) * (D_inch/(3.29546*H_inch))^1.5;
% F_est = rho * S * (Ve.^2 - Ve*V0) * (D_inch/(3.29546*H_inch))^1;



prop = DATA.propellers.p5040x2;
exper = DATA.experiments.racestar_br2205;
% F_est = Gabriel_Stampes(exper.N, prop, rho, V0);
F_est = Gaurang(exper.N, prop, rho, V0);

draw(exper.N, F_est, exper.F)


function draw(N, F_est, F)
    figure(get_uniq_number());
    plot(N, F_est);
    hold on;
    plot(N, F);
    grid on
    xlabel('rpm')
    ylabel('F')
    legend('F_{est}', 'F')
end

function new_numb = get_uniq_number()
    persistent i;
    if isempty(i)
        i = 0;
    end
    i = i + 1;
    new_numb = i;
end
