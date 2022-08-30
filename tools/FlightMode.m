classdef FlightMode
    properties
    %gen: variables: lead x y
        mode_ = struct('x', 0, 'y', 0);
        
    end
    methods
        function mode = FlightMode(vec)
            mode = mode.setX(vec(1));
            mode = mode.setY(vec(2));
        end
        function res = export(mode)
            res = [mode.getX(); mode.getY()];
        end
        
        %gen: setters
        function mode = setX(mode, val)
            mode.mode_.('x') = val;
        end
        function mode = setY(mode, val)
            mode.mode_.('y') = val;
        end
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