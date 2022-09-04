function [w, err, y2] = quadratic_regression(x, y)

% функция для построения матрицы подстановок
f = inline('[x.^0, x, x.^2]','x');
A = f(x);             % матрица подстановок есть функция
                      % значений свободной переменой
w = (A'*A)\(A'*y)    % решить нормальное уравнение
y2 = A*w;             % восстановить зависимую переменную
r = y-y2;             % найти вектор регрессионных остатков
err = r'*r            % подсчитать ошибку