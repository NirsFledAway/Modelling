clc
clear all

global n;


open('lab_2');


data = sim('lab_2');

x_goal = data.X_goal.Data;
y_goal = data.Y_goal.Data;
z_goal = data.Z_goal.Data;

x = data.X.Data;
y = data.Y.Data;
z = data.Z.Data;


r = data.r.Data;
r_goal = data.r_goal.Data;

% n=0;
% for i=1:1:length(r)
%     if (abs(r(i)-r_goal(i))<=20)
%         n=i;
%         break
%     end
% end
% n
% 
% x_goal = x_goal(1:n);
% y_goal = y_goal(1:n);
% z_goal = z_goal(1:n);

% x = x(1:n);
% y = y(1:n);
% z = z(1:n);


figure(1)
clf
plot(x_goal,y_goal,'o')
hold on;
plot(x,y)
hold on;
xlabel('X, м')
ylabel('Y, м')
title('Однолучевой метод наведения')





% h = round(length(x)/10)
% 
% for i=1:h:length(x)
%     plot3([x(i) x_goal(i)],[y(i) y_goal(i)],[z(i) z_goal(i)])
%     hold on;
% end
% 
% 
% ylim([-3000 3000]);
% zlim([-3000 3000]);

