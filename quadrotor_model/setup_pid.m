function setup_pid(mode_vec)
if needChange(mode_vec) == 0
    return
end

Modes = get_pid_koeffs()

flight_mode = struct();
mode = FlightMode(mode_vec);

mode_x = mode.get("x"); mode_y = mode.get("y");

if [mode_x mode_y] == [0 0]
    flight_mode = Modes.Stab_fast;
elseif [mode_x mode_y] == [1 1]
    flight_mode = Modes.Beard_targeting;
else
    flight_mode = Modes.Beard_targeting;
    flight_mode.Reg.PID1 = Modes.Stab_gently.Reg.PID1;
    if mode_x == 2
        %     полет по Берду по ОХ
        flight_mode.Reg.PID2 = Modes.Stab_gently.Reg.PID2;
    end
    if mode_y == 2
        flight_mode.Reg.PID3 = Modes.Stab_gently.Reg.PID3;
    elseif mode_y == 3
        flight_mode.Reg.PID3 = Modes.Stab_gently.Reg.PID3_speed;
    end
end

% if mode == 0
%     flight_mode = Modes.Stab_fast;
% elseif mode == 2
%     flight_mode = Modes.Beard_targeting;
% else
%     flight_mode = Modes.Beard_targeting;
%     flight_mode.Reg.PID1 = Modes.Stab_gently.Reg.PID1;
%     if mode == 3 || mode == 5 || mode == 6
% %     полет по Берду по ОХ
%         flight_mode.Reg.PID2 = Modes.Stab_gently.Reg.PID2;
%     end
%     if mode == 4 || mode == 5
%         flight_mode.Reg.PID3 = Modes.Stab_gently.Reg.PID3;
%     end
%     if mode == 6
%         flight_mode.Reg.PID3 = Modes.Stab_gently.Reg.PID3_speed;
%     end
% end


k_theta = flight_mode.Reg.PID1';    % make column
k_x     = flight_mode.Reg.PID2';
k_y     = flight_mode.Reg.PID3';

S = get_model_params();
% set_param([S.reg_theta_path 'GainP'],'Gain', string(val));
P = 'P1'; I = 'I1'; D = 'D1';
setup_pid_gain(S.reg_theta_path, [P; I; D], k_theta);
setup_pid_gain(S.reg_x_path, [P; I; D; 'P2'], [k_x; k_x(1)]);
setup_pid_gain(S.reg_y_path, [P; I; D], k_y);

function need = needChange(val)
    persistent mm
    if isempty(mm)
        mm = [-1 -1]';
    end
    if mm == val
        need = 0;
    else
        mm = val(:,:);
        need = 1;
    end


function S = get_model_params()
    model_name = 'run_quadrotor_2020a/';
    reg_set_path = [model_name 'REG/'];
    S.reg_theta_path = [reg_set_path 'Тангаж (theta)/'];
    S.reg_x_path = [reg_set_path 'X/'];
    S.reg_y_path = [reg_set_path 'Y/'];

function setup_pid_gain(subsystem, labels, values)
    for i = 1:length(labels)
        set_param([subsystem labels(i, :)],'Gain', string(values(i)));
    end