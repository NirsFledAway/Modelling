clc; clear variables; close all;

state_raw = load('state.mat')
sensors_raw = load('sensors.mat')
time = sensors_raw.ans.Time;
sensors = sensors_raw.ans.Data(:, :);
state = state_raw.ans.Data(:, :);
time_contin = state_raw.ans.Time;

Gyroscope = sensors(4:6, :)';
Gyroscope = [Gyroscope(:, 1) Gyroscope(:, 3) -Gyroscope(:, 2)];
Accelerometer = sensors(1:3, :)';
Accelerometer = [Accelerometer(:, 1) Accelerometer(:, 3) -Accelerometer(:, 2)];
Magnetometer = sensors(7:9, :)';
Magnetometer = [Magnetometer(:, 1) Magnetometer(:, 3) -Magnetometer(:, 2)];

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
dt = 0.001;
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

AHRS = MadgwickAHRS('SamplePeriod', 1/1000, 'Beta', 0);
% AHRS = MahonyAHRS('SamplePeriod', 1/1000, 'Kp', 0);

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