classdef FlightMode
    properties
    %gen: variables: lead x y
    mode_ = struct(
        %gen: varList
    );
        mode_ = struct('x', 0, 'y', 0);
        
    end
    methods
        function mode = FlightMode(vec)
        %gen: setValues
            mode = mode.setX(vec(1));
            mode = mode.setY(vec(2));
        end
        function res = export(mode)
            %gen: buildVector
        end
        
        %gen: setters
        function value = get(mode, key)
            switch key
                %gen: get_by_key
                otherwise
                    value = 0;
            end
        end
        %gen: getters
    end
end