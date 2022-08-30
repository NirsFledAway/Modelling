clc; close all;
f = 2000
W = tf([1], [1/(f) 1])
bode(W)
grid on
grid minor