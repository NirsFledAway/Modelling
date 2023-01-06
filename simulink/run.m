%%
sim("simulink/run_quadrotor_2020a.slx", "StopTime", '20', "Debug", "off")

%%
close all
W = tf([15], [1 15])
step(W)
%%
Vg = [10 0 2]';  
Vb = getRotationMatrix([phi theta psi]) * Vg
V_xy = sqrt(Vb(1)^2 + Vb(2)^2);
% beta = rad2deg(acos(V_xy/norm(Vg))) * (-1) * sign(Vb(3)) 
beta_beard_1 = -rad2deg(asin(Vb(3)/norm(Vg)))
% beta_beard_2 = -rad2deg(atan(Vb(3)/V_xy))
% beta_old_fuck = atan2(-Vb(3), Vb(1))

%%
clc
des = [179 -179 0 5 -5 343 -343]
ang = [-179 179 5 0 0 -343 343]
des = f2(des)
ang = f2(ang)
err = des - ang
err_ans = [-2 2 -5 5 -5 -34 34]
% f(des - ang)
err1 = err + 360 * sign(180-err).*(abs(err)>180)

assert(sum(err1 == err_ans) == length(err1))
%%
a = [0 10 -10 179 -179 190 -190 359 -359 361 -380]
a
f2(a)
%%
clc
clear variables
close all
t = 0:0.01:10;
t_0 = 7-2;
r0 = [
    20;
    0;
    20
];
v0 = [
    -3;
    0;
    5
];
a0 = [
    0;
    0;
    0
]
R = 50;
ff = 1/20;
k_line = v0(3)/v0(1);
phi = atan(k_line);
r = zeros(3, length(t));
r_d = r0 + v0*(t_0) + a0*(t_0).^2/2
% r_d = [88.57; 0; 54.98]

for i=1:length(t)
    r(:, i) = r0 + v0*t(i) + a0*t(i).^2/2;
    if t(i) >= t_0
        r(:, i) = [
            R*(cos(2*pi*ff*(t(i)-t_0)+(3/2*pi + phi))) + r_d(1) - R*cos(3*pi/2+phi);
            0;
            R*(sin(2*pi*ff*(t(i)-t_0)+(3/2*pi + phi))) + r_d(3) - R*sin(3*pi/2+phi)
        ];
    end
end
plot(r(1,:), r(3,:))
%%
close all
t = 0:0.01:15;
t_0 = 0;
r = zeros(3, length(t));
phi = deg2rad(-60);
for i=1:length(t)
    r(:, i) = r0 + v0*t(i) + a0*t(i).^2/2;
    if t(i) >= t_0
        r(:, i) = [
            R*(cos(2*pi*ff*(t(i)-t_0) + phi)+ cos(phi));
            0;
            R*(sin(2*pi*ff*(t(i)-t_0) + phi)+ sin(phi))
        ];
    end
end
figure
plot(r(1,:), r(3,:))
%%
% матрица поворота (g->b) {из Земной нормальной в связанную СК}
function R = getRotationMatrix(angles)
    phi = 1; t = 2; psi = 3; %indices
    s = sin(angles);
    c = cos(angles);
    R = [
        c(t)*c(psi) s(t)    -c(t)*s(psi);
        -c(phi)*s(t)*c(psi)+s(phi)*s(psi) c(phi)*c(t) c(phi)*s(t)*s(psi)+s(phi)*c(psi);
        s(phi)*s(t)*c(psi)+c(phi)*s(psi)  -s(phi)*c(t)    -s(psi)*s(t)*s(phi)+c(psi)*c(phi);
    ];
end

function err1 = f(err)
    if err > 180
        err1 = err - 360;
    elseif err < -180
        err1 = err + 360;
    else
        err1 = err;
    end
%         err1 = (err - sign(err)*360);
end

function a = f2(a)
    a = a - sign(a).*360.*(abs(a) > 180);
%     for i=1:length(a)
%         if abs(a(i)) > 180
%             a(i) = a(i) - sign(a(i)).*360;
%         end
%     end
%     a(abs(a) > 180) = a - sign(a).*360
end