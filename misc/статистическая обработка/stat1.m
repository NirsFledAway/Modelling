clc; clear variables; close all; aerosonde_parameters;
model = "run_quadrotor_2020a";
load_system(model);
experiments_count = 100;
%%
data = cell(experiments_count, 1);

% accel_seed = [23341 333 555];
% gyro_seed = [23341 333 555];
% wind_seed = 0;
simIn(1:experiments_count) = Simulink.SimulationInput(model);
for i=1:experiments_count
    simIn(i) = setVariable(simIn(i), 'MAV.Sensors.Accel.Seed', [1 1 1] * (1 + i));
    simIn(i) = setVariable(simIn(i), 'MAV.Sensors.Gyro.Seed', [1 1 1] * (100 + i));
    simIn(i) = setVariable(simIn(i), 'MAV.Env.wind_seed', 200 + i);
    simIn(i) = setModelParameter(simIn(i), "StopTime", "10");
    simIn(i) = setModelParameter(simIn(i), "Debug", "off");
end
simout = parsim(simIn);
%%
% save('batch_simulation_data_100_position_10sec.mat', 'simout', 'experiments_count');
% parfor i=1:experiments_count
%     res = sim("simulink/run_quadrotor_2020a.slx", "StopTime", '2', "Debug", "off");
%     outputs = res.yout.signals;
%     data{i} = outputs;
% end
%%
load('batch_simulation_data_200_landings_14sec.mat')
%% Experiment 1. Время и точность посадки на подвижную цель
% experiments_count = 50;
t_end = zeros(experiments_count, 1);
delta_r = zeros(experiments_count, 1);
energy = zeros(experiments_count, 1);
for i = 1:experiments_count
    t_end(i) = simout(i).yout.time(end);

    signals = simout(i).yout.signals;
    state = signals(1).values;
    target = signals(5).values;
    motors = signals(4).values;

    r = state(end, [1, 3]);
    r_target = target(end, [1, 3]);
    delta_r(i) = sqrt(sum((r - r_target).^2));

    e = 0;
    for j = 1:length(simout(i).yout.time)
        e = e + sum(motors(j, :).^2);
    end
    energy(i) = e;
end

close all
clc
figure();
subplot(1,3,1)

histogram(t_end, 20)
title("Время посадки")
xlabel("t, c")
disp("t_end")
stat_analysis(t_end);

subplot(1,3,2)
histogram(delta_r, 20)
title("Промах")
xlabel("\Delta r, м")
disp("delta_r")
% stat_analysis(delta_r);

subplot(1,3,3)
histogram(energy, 50)
title("Энергозатраты")
xlabel("unit")
disp("energy")
% stat_analysis(energy);

%% Стабилизация скорости
load('batch_simulation_data_200_speed_5ms_10sec.mat')
delta_v_max = zeros(experiments_count, 1);
energy = zeros(experiments_count, 1);
t_cut = 2;
v_target = 5;

for i = 1:experiments_count
    t = simout(i).yout.time;

    signals = simout(i).yout.signals;
    state = signals(1).values(t>t_cut, :);
    target = signals(5).values(t>t_cut, :);
    motors = signals(4).values(t>t_cut, :);

    delta_v_max(i) = max(state(:, 4) - v_target);

    e = 0;
    for j = 1:size(motors, 1)
        e = e + sum(motors(j, :).^2);
    end
    energy(i) = e;
end
i = 5;
signals = simout(i).yout.signals;
state = signals(1).values(t>t_cut, :);
target = signals(5).values(t>t_cut, :);
motors = signals(4).values(t>t_cut, :);
delta_v_single = state(:, 4) - v_target;

close all
clc
figure();
subplot(1,3,1)
histogram(delta_v_max, 20)
title("Максимальная ошибка скорости")
xlabel("\Delta v, м/c")
disp("delta_v_max")
stat_analysis(delta_v_max);

subplot(1,3,2)
histogram(delta_v_single, 20)
title("Ошибка скорости одного опыта")
xlabel("\Delta v, м")
disp("delta_v_single")
stat_analysis(delta_v_single);

subplot(1,3,3)
histogram(energy, 20)
title("Энергозатраты")
xlabel("units")
disp("energy")
stat_analysis(energy);

%% Стабилизация положения
load('batch_simulation_data_100_position_10sec.mat')
delta_r_max = zeros(experiments_count, 1);
delta_r_end = zeros(experiments_count, 1);
delta_v_max = zeros(experiments_count, 1);

for i = 1:experiments_count
    t_end(i) = simout(i).yout.time(end);

    signals = simout(i).yout.signals;
    state = signals(1).values;

    r = state(end, 1);
    delta_r_end(i) = r - 0;
    delta_r_max(i) = max(state(:, 1) - 0);
    delta_v_max(i) = max(state(:, 4) - 0);
end

close all
clc
figure();
subplot(1,3,1)
histogram(delta_r_end, 20)
title("Конечное смещение")
xlabel("\Delta x, м")
disp("delta_r_end")
stat_analysis(delta_r_end);

subplot(1,3,2)
histogram(delta_r_max, 20)
title("Максимальное смещение")
xlabel("\Delta r, м")
disp("delta_r_max")
stat_analysis(delta_r_max);

subplot(1,3,3)
histogram(delta_v_max, 20)
title("Максимальная ошибка скорости")
xlabel("\Delta v, м/с")
disp("delta_v_max")
stat_analysis(delta_v_max);

%% Копипаста с Чулина для адаптации

function stat_analysis(y)

x_min = min(y)
x_max = max(y)
R = x_max - x_min

mx = mean(y) % Выборочное среднее
m2x = mean((y - mx).^2) % Выборочная дисперсия
s2x=sqrt(m2x) % СКО - среднеквадратичное отклонение
sx = std(y) % Стандартное отклонение, то же самое что СКО

m3x = mean((y - mx).^3) % Выборочный центральный момент 3 порядка
m4x = mean((y - mx).^4) % Выборочный центральный момент 4 порядка

gamma1 = m3x/(m2x^1.5) % Асимметрия, с. 83 книги

% fprintf('  Эксцесс (для норм.распределения 3)\n')
gamma2 = m4x/(m2x^2) - 3  % Эксцесс, с. 86 книги

% fprintf('  Контрэксцесс (для норм.распределения ~0.577)\n')
gamma05 = m2x/sqrt(m4x)  % Контрэксцесс

l = length(y);
t_95 = 1.96;
delta_x_095 = t_95 * s2x / sqrt(l)
% input('Нажмите ENTER');
% %Для протокола
% if nmXY=='x'
%     mxX=mx;
%     m2xX=m2x;
%     sxX=sx;
%     gamma1X=gamma1;
%     gamma2X=gamma2;
%     gamma05X=gamma05;
% else 
%     mxY=mx;
%     m2xY=m2x;
%     sxY=sx;
%     gamma1Y=gamma1;
%     gamma2Y=gamma2;
%     gamma05Y=gamma05;
% end
% 
% %Гистограмма
% kHIST1=3.31*log10(nWYB)+1
% kHIST2=5*log10(nWYB)
% kHIST=round((kHIST1+kHIST2)/2)
% %fprintf('Число участков гистограммы %d\n', kHIST)
% %nmYES=input('Хотите изменить(введите y, если ДА, или n, если НЕТ):','s');
% %if nmYES=='y'
%     %fprintf('Рекомендуемый диапазон %d - %d\n', round(kHIST1), round(kHIST2))
%     %kHIST=input('Число участков гистограммы?');
% %end
% [NHIST, yHIST] = hist(RBTGL,kHIST);
% NHISTnWYB=NHIST/nWYB; %Относительные частоты
% hHIST=(yHIST(kHIST)-yHIST(1))/kHIST; %Длина разряда гистограммы
% hHIST1=yHIST(2)-yHIST(1); %Длина разряда гистограммы
% hHISTn=(max(RBTGL)-min(RBTGL))/kHIST; %Длина разряда гистограммы
% %Теоретические вероятности
% fxH = normpdf(yHIST,mx,sx)*hHIST;
% %Хи-квадрат
% hi2=nWYB*sum((NHISTnWYB-fxH).^2/fxH)
% 
% bar(yHIST, NHIST)
% %Диапазон "3-сигма" с шагом 0.1*CKO
% ksi = (-3.0:0.1:3.0)*sx + mx;
% %Теоретические вероятности
% fx = normpdf(ksi,mx,sx)*hHIST;
% figure;
% plot(ksi,fx, yHIST, NHISTnWYB)
end