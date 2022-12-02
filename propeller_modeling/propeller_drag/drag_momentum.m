clc; close all; clear variables;

A = [
% PWM Weight Current
1200    4.77  0;
1250    13.19   1.47;
1300    22.68   2.11;
1350    30.85   3.05;
1400    39.70   3.9;
1450    50.4    4.9;
1500    57.44   6.0;
1550    73.2    7.4;
1600    84.14   8.95;
1650    90.83   10.67;
1700    111.3 0;
1750    130.6 0;
1800    143.15 0;
1850    149.6 0;
1900    155.2 0;
1950    172.0 0;
2000    184.4 0;
];
emax_data = [
% thrust current RPM
300	4.2	15700;
500	8.3	19700;
700	13.4	23420;
1290	28.8	33000;
];

pwm = A(:,1);
M = A(:,2);
curr = A(:, 3);

emax_thrust = emax_data(:,1);
emax_curr = emax_data(:,2);
emax_rpm = emax_data(:, 3);

%%
close all
figure;
subplot(3, 1, 1)
plot(emax_rpm, emax_curr);
xlabel('rpm');
ylabel('curr');
hold on;

subplot(3, 1, 2)
plot(emax_rpm, emax_thrust);
xlabel('rpm');
ylabel('Thrust');

subplot(3, 1, 3)
plot(emax_curr, emax_thrust);
xlabel('Curr');
ylabel('Thrust');

%%
figure;
plot(curr, M);
figure;
plot(emax_rpm, emax_curr);

% x = emax_rpm;
% y = emax_curr;
[k, b, err, y1] = linear_regression(emax_rpm, emax_curr);
figure;
scatter(x, y); hold on;
plot(x, y1);
legend('real', 'approximated');

%%
[k, b, err, y1] = linear_regression(pwm(1:9), curr(1:9))
curr(10:end) = pwm(10:end)*k + b
curr

%%
close all
[w, err, y2] = quadratic_regression(emax_rpm(1:3),emax_thrust(1:3));
figure
scatter(emax_rpm, emax_thrust);hold on;
plot(emax_rpm, w(1)+emax_rpm*w(2) + emax_rpm.^2*w(3));
legend('real', 'approx')
xlabel('rpm');
%%
close all
figure
plot(emax_curr, emax_rpm);
hold on
plot(curr, (pwm-1000)/1000*40000)
legend('emax rpm', 'pwm');
xlabel('current, A');

%%
close all

x = N;
momentum = (M/1000 * 9.8) * (66.5/1000);
y = momentum;

[w, err, y2] = quadratic_regression(x,y);


% figure;
% plot(pwm, rpm); hold on;
% plot(pwm, rpm_approx);

figure;
scatter(x, y); hold on;
% plot(x, y1);
plot(x, y2);
%%
close all
% [k, b, err, y1] = linear_regression(emax_rpm, emax_curr);
% curr = k*rpm + b
% rpm = (curr - b) / k
% x = (pwm - 1000)/1000;
EMAX_MAX_RPM = 33000;
MIN_PWM = 1000; MAX_PWM = 2000;
rpm_approx = (pwm-1000)/(MAX_PWM-MIN_PWM) * EMAX_MAX_RPM;
x = rpm_approx;
% x = x / 60;
y = M;
momentum = (M/1000 * 9.8) * (66.5/1000);
y = momentum
% y = y * 1000;
% [k, b, err, y1] = linear_regression(x, y);
% x = curr;
% x = pwm;
% y = rpm;
[w, err, y2] = quadratic_regression(x,y);
fuck = [x'./1e3; y']


% figure;
% plot(pwm, rpm); hold on;
% plot(pwm, rpm_approx);

figure;
scatter(x, y); hold on;
% plot(x, y1);
% plot(x, y2);
x = x;
% plot(x, -0.01 + 7.8453e-09*(2*pi*x/60).^2 );
% plot(x, 0.00000000007009*x.^2 + 0.00000160889000*x - 0.00778245010321);
    % plot(x, 0.00000000008730*x.^2+0.00000084282100*x-0.00027874970727);
hold on
c = 8.7300e-11;
a = -4.8272e+03;
b = -0.0023;
% % 8.7e11*(x+4800) - 0.0023

% plot(x, c*(x-a).^2+b);
% plot(x, c*(x-a).^2);
% Аппроксимация с точкой 0 
c = 0.0000000000880085;
a = 0.0000008149303785;
b = -0.0000350181946204;
x1 = 0:10:40000;
% plot(x1, c*x1.^2 + a*x1 + b)
% с центром параболы в (0, 0) без сдвигов
% c = 1.154077248e-10;
c = 0.0000000001066078;
b = 0.0077448458147488;
plot(x1, c*x1.^2 + b, 'LineWidth', 2)
% plot(x1, 1.221251133648546e-10*x1.^2)
% plot(x1, 1.221255238644330e-10*x1.^2, 'LineWidth', 2)
legend({
'Эксперимент'
% 'quadratic approx'
% 'podbor'
% 'podbor1'
% 'podbor2'
% 'podbor_2.1'
'Аппроксимация'

% 'Без сдвига'
});
xlabel('RPM_{approx}');
xlim([0; 33000]);
ylabel('M, Н\cdot м');
% ylim([0; 0.15]);
% c*(x-a)^2+b = cx^2 - 2*a**c*x + (c*a^2 + b) = 
% c = 0.00000000008730
% a = 
%%
rpm_approx
sprintf("%d ", rpm_approx.^2)
sprintf("%f ", momentum)
%%
% a*x^2 + b*x + c = f*(x-e)^2 + g
% -----           = f*x^2 - 2*e*f*x + f*e^2 + g
% f = a
% b = -2*e*a => e = -b/2a
% c = g + a*b^2/4a^2 = g + b^2/4a => g = c - b^2/4a
%%
A = [
    162	3;
236	5;
311	7;
374	9.1;
439	11;
490	13;
548	15.3;
611	17.3;
712	20.7;
];
thrust = A(:,1);
curr = A(:,2);
figure;
plot(curr, sqrt(thrust));

%%
sprintf("%d ", rpm_approx)
sprintf("%f ", momentum)
%%
figure; hold on;
plot(rpm_approx.^2, momentum)
plot(x1.^2, 1.221255238644330e-10*x1.^2)
c = 0.0000000001066078;
b = 0.0077448458147488;
plot(x1.^2, c*x1.^2 + b)
c = 8.7300e-11;
a = -4.8272e+03;
b = -0.0023;
% % 8.7e11*(x+4800) - 0.0023

plot(x1.^2, c*(x1-a).^2+b);
legend([
    "god-shit"
    "non-drift"
    "drift"
    "old double-drift"
]);
%%
rpm = [8250 9900 11550 13200 14850 16500 18150 19800 21450 28050];
y = [0.008596 0.014781 0.020105 0.025872 0.032846 0.037434 0.047704 0.054834 0.059194 0.097494]
f = @(c) average_quadraric_error(c*rpm_approx.^2, momentum);
f(5.000000002500000e-07)
%%
C_T_opt = dichotomi_optimization(f, [1e-20, 1e-6], 1e-20)

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
