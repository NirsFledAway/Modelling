clc; clear variables; close all;

state_raw = load('state.mat');
sensors_raw = load('sensors.mat');
control_raw = load('control.mat');
time = sensors_raw.ans.Time;
sensors = sensors_raw.ans.Data(:, :);
state = state_raw.ans.Data(:, :);
time_contin = state_raw.ans.Time;
time_control = control_raw.ans.Time;
control = control_raw.ans.Data(:,:);

Gyroscope = sensors(4:6, :)';
Accelerometer = sensors(1:3, :)';
Magnetometer = sensors(7:9, :)';
%%
state_noise_raw = load('state_noise.mat');
time_contin_noise = state_noise_raw.ans.Time;
state_noise = state_noise_raw.ans.Data(:, :);

control_noise_raw = load('control_noise.mat');
time_control_noise = control_noise_raw.ans.Time;
control_noise = control_noise_raw.ans.Data(:,:);
%%
Gyroscope = [Gyroscope(:, 1) Gyroscope(:, 3) -Gyroscope(:, 2)];
Accelerometer = [Accelerometer(:, 1) Accelerometer(:, 3) -Accelerometer(:, 2)];
Magnetometer = [Magnetometer(:, 1) Magnetometer(:, 3) -Magnetometer(:, 2)];

%%
close all
lim = [0 2]
figure('Name', 'Psi stab')
title('Ideal process')
subplot(2,2,1)
title('Ideal process')
plot(time_contin, rad2deg(state(:, 9)), 'b')
hold on;
plot(time_contin, ones(size(time_contin))*90)
legend('current', 'desired')
xlabel('t, c')
ylabel('\psi, ^\circ')
title('Ideal process')
xlim(lim)
ylim([0 95])

subplot(2,2,3)
plot(time_control, control(:, 3))
xlabel('t, c')
ylabel('u_y')
legend('u_y control')
xlim(lim)

subplot(2,2,2)

plot(time_contin_noise, rad2deg(state_noise(:, 9)), 'b')
hold on;
plot(time_contin_noise, ones(size(time_contin_noise))*90)
legend('current', 'desired')
xlabel('t, c')
ylabel('\psi, ^\circ')
title('With noise')
xlim(lim)
ylim([0 95])

subplot(2,2,4)
plot(time_control_noise, control_noise(:, 3))
xlabel('t, c')
ylabel('u_y')
legend('u_y control')
xlim(lim)
%%
% Стаб малых скоростей по х
close all;
figure();
subplot(2,2,1)
plot(time_contin, state(:, 1), 'r', 'LineWidth', 2); hold on;
plot(time_contin_noise, state_noise(:, 1), 'b');
xlabel('t, c')
ylabel('x, м')

subplot(2,2,2)
plot(time_contin, state(:, 4), 'r', 'LineWidth', 2);hold on;
plot(time_contin_noise, state_noise(:, 4), 'b');
xlabel('t, c')
ylabel('V_x, м/с')
legend('w/o noise', 'with noise')

subplot(2,2,3)
plot(time_contin, state(:, 8), 'r', 'LineWidth', 2);hold on;
plot(time_contin_noise, state_noise(:, 8), 'b');
xlabel('t, c')
ylabel('\theta, град')

subplot(2,2,4)
plot(time_control, control(:, 4), 'r', 'LineWidth', 2);hold on;
plot(time_control_noise, control_noise(:, 4), 'b');
xlabel('t, c')
ylabel('u_z')

%%
% Стаб высоты
close all;
figure();
subplot(2,2,1)
plot(time_contin, state(:, 2), 'r'); hold on;
plot(time_contin_noise, state_noise(:, 2), 'b');
xlabel('t, c')
ylabel('y, м')
xlim([0 3])

subplot(2,2,2)
plot(time_contin, state(:, 5), 'r');hold on;
plot(time_contin_noise, state_noise(:, 5), 'b');
xlabel('t, c')
ylabel('V_y, м/с')
legend('w/o noise', 'with noise')
xlim([0 3])

% subplot(2,2,3)
% plot(time_contin, state(:, 8), 'r', 'LineWidth', 2);hold on;
% plot(time_contin_noise, state_noise(:, 8), 'b');
% xlabel('t, c')
% ylabel('\theta, град')

subplot(2,2,4)
plot(time_control, control(:, 1), 'r');hold on;
plot(time_control_noise, control_noise(:, 1), 'b');
xlabel('t, c')
ylabel('u_T')
xlim([0 3])


%%
close all;
figure('Name', 'Euler angles real')
hold on;
plot(time_contin, rad2deg(state(:, 7:9)))
legend("phi", "theta", "psi")
% %%
% figure('Name', 'Sample time')
% plot(time(1:end-1), time(2:end) - time(1:end-1))

q = [1 0 0 0];
dt = 1/500;
quaternion = zeros(length(time), 4);
for i=1:length(time)
    w = [0 Gyroscope(i, 1) Gyroscope(i, 2) Gyroscope(i, 3)];
    q_dot = 0.5 * quaternProd(q, w);
    q = q + q_dot * dt;
    q = q/norm(q);
    quaternion(i,:) = q;
end
euler = quatern2euler(quaternConj(quaternion));
euler = [euler(:, 1), euler(:, 2), -euler(:, 3)];
euler = rad2deg(euler);
figure('Name', 'test1');
plot(time, euler);

%%
figure('Name', 'Sensor Data'); hold on;
plot(time, rad2deg(Gyroscope(:,1)), 'r');
plot(time, rad2deg(Gyroscope(:,2)), 'g');
plot(time, rad2deg(Gyroscope(:,3)), 'b');
xlabel('t, с');
ylabel('Угловая скорость, град/с');
legend('X', 'Y', 'Z');
title('Гироскоп');
%% Import and plot sensor data

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm

% AHRS = MadgwickAHRS('SamplePeriod', 1/1000, 'Beta', 0.05);
AHRS = MahonyAHRS('SamplePeriod', 1/1000, 'Kp', 0.5);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
%     AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
        AHRS.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end
%%
w = [0 0 deg2rad(60) 0]
q = [1 0 0 0]
dt = 0.1;
for i=1:round(1/dt)
    q_dot = 0.5 * quaternProd(q, w);
    q = q + q_dot * dt;
    q = q/norm(q);
end
euler = quatern2euler(quaternConj(q)) * (180/pi)
%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, -euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% End of script