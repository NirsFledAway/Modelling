clc; close all; clear variables;

A = [0 1 0]';
angles = deg2rad([30 0 0])

R = getRotationMatrix(angles)';
A1= R * A;
A1
figure();
% x = 0:0.01:A1(1);
% y = 0:0.01:A1(2);
% z = 0:0.01:A1(3);
x = A1(1); y = A1(2); z = A1(3);
plot3([0 x], [0 y], [0 z], 'r', 'LineWidth', 5);
grid on
hold on;
plot3([-1 1], [0 0], [0 0], 'g', 'LineWidth', 2)
plot3([0 0], [-1 1], [0 0], 'b', 'LineWidth', 2)
plot3([0 0], [0 0], [-1 1], 'm', 'LineWidth', 2)
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])

function R = getRotationMatrix(angles)
    phi = 1; t = 2; psi = 3; %indices
    s = sin(angles);
    c = cos(angles);
    R = [...
        c(psi)*c(t), s(phi)*s(psi) + c(phi)*c(psi)*s(t), c(psi)*s(phi)*s(t) - c(phi)*s(psi); ...
        -s(t),      c(phi)*c(t),                  c(t)*s(phi);                     ...
        c(t)*s(psi), c(phi)*s(psi)*s(t) - c(psi)*s(phi),    c(phi)*c(psi) + s(phi)*s(psi)*s(t)   ...
    ];
end