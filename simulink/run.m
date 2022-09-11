init_script;
need_wind = 1;
% if need_wind
%     setup_block_param("Wind On/Off", "Port", 1);
% end
% %
% get_data_list('run_quadrotor_2020a/"Wind On/Off');
%%
% Wind_ON = Simulink.Parameter(1);
% Wind_ON = 0;
sim("simulink/run_quadrotor_2020a.slx", "TimeOut", 30)

%%
function setup_block_param(labels, param, value)
    set_param(['run_quadrotor_2020a/' labels(:)], param, string(value));
end