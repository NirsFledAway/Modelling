classdef Utils
    methods (Static)
        function m = inch2met(inch)
            m = inch * 2.54 / 100;
        end

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
        
        function [C, screen_size] = getCenter()
           set(0,'units','pixels')  
            %Obtains this pixel information
            screen_size = get(0,'screensize')
            C = screen_size/2;
        end
    end
end