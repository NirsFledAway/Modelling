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
plot(x, y2);
x = x;
% plot(x, -0.01 + 7.8453e-09*(2*pi*x/60).^2 );
% plot(x, 0.00000000007009*x.^2 + 0.00000160889000*x - 0.00778245010321);
plot(x, 0.00000000008730*x.^2+0.00000084282100*x-0.00027874970727);
hold on
c = 8.7300e-11;
a = -4.8272e+03;
b = -0.0023;
% % 8.7e11*(x+4800) - 0.0023

plot(x, c*(x-a).^2+b);
plot(x, c*(x-a).^2);
legend('real', 'quadratic approx', 'podbor', 'podbor1', 'podbor2');
xlabel('rpm_approx');
xlim([0; 33000]);
% ylim([0; 0.15]);
% c*(x-a)^2+b = cx^2 - 2*a**c*x + (c*a^2 + b) = 
% c = 0.00000000008730
% a = 

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
