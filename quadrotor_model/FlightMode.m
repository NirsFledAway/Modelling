% classdef FlightMode
%     properties (SetAccess = public)
%         mode double = 1
%     end
%     methods (Static)
%         function m = inch2met(inch)
%             m = inch * 2.54 / 100;
%         end

%         function setFlight(val)
%             mode = val;
%         end
%     end
% end

FlightMode1 = struct('x', 0, 'y', 0, 'main', 0);