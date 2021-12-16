clc; clear variables; close all;

%% load data
data; utils;
% rho = 1.2041;
% rho = 1.18
% % rho = 1.225;
% V0 = 0;

prop = DATA.propellers.p5040x2;
exper = DATA.experiments.racestar_br2205;

prop = DATA.propellers.dalprope_cyclone_5040x3;
exper = DATA.experiments.emax_eco_ii_2306__5040;

% exper = DATA.experiments.emax_rs2205_5045x3;
% prop = DATA.propellers.p5045x3;

%% calc forces
% F_est = Gabriel_Stampes(exper.N, prop, rho, Va);
% F_est = Gaurang(exper.N, prop, rho, Va);
F_est = Beard_UAV(exper.N, prop, rho, Va, 0.5076);
% 
draw(exper.N, F_est, exper.F)

%% Подбираем C_T для Beard
C_T_opt = dichotomi_optimization(@f, [0, 1], 0.000001)
function y = f(C_T)
%     global exper prop rho F;
    data; utils;
    prop = DATA.propellers.dalprope_cyclone_5040x3;
    exper = DATA.experiments.emax_eco_ii_2306__5040;
    
    F_est = Beard_UAV(exper.N, prop, rho, Va, C_T);
    y = average_quadraric_error(exper.F, F_est);
end


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

function x_min = dichotomi_optimization(f, K0, e)
    a = K0(1); b = K0(2);
    delta = e/2;
    while b - a > 2*e
        x1 = (a + b - delta)/2;
        x2 = (a + b + delta)/2;
        if f(x1) <= f(x2)
            b = x2;
        else
            a = x1;
        end
    end
    x_min = (a + b)/2;
end

function err = average_quadraric_error(A, B)
    err = sum((A - B).^2) / (length(A)-1);
end
