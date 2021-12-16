inch2met = @(val) val*2.54/100;

% function x_min = dichotomi_optimization(f, K0, e)
%     a = k0(1); b = k0(2);
%     delta = e/2;
%     while b - a > 2*e
%         x1 = (a + b - delta)/2;
%         x2 = (a + b + delta)/2;
%         if f(x1) <= f(x2)
%             b = x2;
%         else
%             a = x1;
%         end
%     end
%     x_min = (a + b)/2;
% end
% 
% function err = average_quadraric_error(A, B)
%     err = sum((A - B).^2) / (length(A)-1);
% end