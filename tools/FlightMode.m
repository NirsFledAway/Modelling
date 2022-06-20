classdef FlightMode
    properties
        mode_ = struct('x', 0, 'y', 0);
        
    end
    properties(Constant)
        keys_ = ['x'; 'y'];
    end
    methods
        function mode = FlightMode(vec)
%             mode.keys_ = ['x'; 'y'];
            mode = mode.setX(vec(1));
            mode = mode.setY(vec(2));
        end
        function res = export(mode)
            res = [mode.getX(); mode.getY()];
%             res = zeros(length(mode.keys_));
%             for i = 1:length(mode.keys_)
%                res(i) = mode.mode_.(mode.keys_(i));
%             end
        end
        
        function mode = setX(mode, val)
            mode.mode_.('x') = val;
        end
        function mode = setY(mode, val)
            mode.mode_.('y') = val;
        end
%         function mode = set(mode, keys, values)
%             mode.mode_.('x') = values(1);
%             mode.mode_.('y') = values(2);
% %             for i = 1:length(keys)
% %                mode.mode_.(keys(i)) = values(i); 
% %             end
%            for i = 1:length(keys)
%                mode.mode_ = setfield(mode.mode_, keys(i), values(i));
% %               mode.mode_.(mode.keys_(i)) = vec(i); 
%            end
%         end
        function value = get(mode, key)
            switch key
                case 'x'
                    value = mode.getX();
                case 'y'
                    value = mode.getY();
                otherwise
                    value = 0;
            end
        end
        function value = getX(mode)
            value = mode.mode_.('x');
        end
        function value = getY(mode)
            value = mode.mode_.('y');
        end
    end
end