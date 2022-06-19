%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% simulation parameters
%
% mavMatSim
%   - Beard & McLain, PUP 2012
%   - Modification History:
%       12/15/2018 - RWB

SIM.ts_simulation = 0.02;  % smallest time step for simulation
SIM.ts_control = SIM.ts_simulation;
SIM.ts_plotting = 0.1;
SIM.ts_video = 0.1;

SIM.start_time = 0;
SIM.end_time = 30;

SAMPLE_TIME = inf;

model_name = 'run_quadrotor_2020a';
reg_set_path = [model_name '/REG'];
SIM.model_params.reg_theta_path = [reg_set_path '/Тангаж (theta)']

