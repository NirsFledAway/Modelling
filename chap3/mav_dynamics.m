function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,MAV)

switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(MAV);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u,MAV);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(MAV)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);  % Simulink func

%
% initialize the initial conditions
%
x0  = [...
    MAV.pn0;...
    MAV.pe0;...
    MAV.pd0;...
    MAV.u0;...
    MAV.v0;...
    MAV.w0;...
    MAV.phi0;...
    MAV.theta0;...
    MAV.psi0;...
    MAV.p0;...
    MAV.q0;...
    MAV.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
% function sys=mdlDerivatives(t,x,uu, MAV)
% 
% %   Earth coordinates
%     pn    = x(1);
%     pe    = x(2);
%     pd    = x(3);
% %   speed in 'b' coordinates
%     u     = x(4);
%     v     = x(5);
%     w     = x(6);
% %   position quaternions
%     phi    = x(7);
%     theta    = x(8);
%     psi    = x(9);
% %   angular speeds
%     p     = x(10);  % крен
%     q     = x(11);  % тангаж
%     r     = x(12);  % рыскание
% %   inputs
%     f1    = uu(1);  
%     f2    = uu(2);
%     f3    = uu(3);
%     f4 = uu(4);
% 
%     mx    = uu(5);
%     my     = uu(6);
%     mz     = uu(7);
% 
% %     p = x(1:3)
% %     V = x(4:6);
% %     euler = x(7:9);
% %     omega = x(10:12);
% %     f = uu(1:3)
% %     m = uu(4:6);
% 
%     R_g_b = MAV.R_g_b([phi theta psi]); % матрица поворота (g->b) {из Земной нормальной в связанную СК}
% 
%     % Предварительные вычисления
%     f_gravity = R_g_b * [0; MAV.mass*(-MAV.gravity); 0];
%     f_resistance = [0; 0; 0];
%     P_traction = [0; 1; 0] * (f1 + f2 + f3 + f4);   % тяго
%     F_b = f_gravity + f_resistance + P_traction;    % результирующая сил в связанной СК
% 
%     M_gyro = [mx my mz]';  % от винтов, вращение по рЫсканию
%     M_traction = [
%         ( f1 + f4 - (f2 + f3) ) * MAV.radius_z*1e-3;
%         0;
%         ( f1 + f2 - (f3 + f4) ) * MAV.radius_x*1e-3;
%     ];
% %     M_traction = [0 0 0]';
%     M_b = M_gyro + M_traction;
%     % --------
%     
%     
%     % поступательная кинематика
%     p_dot = R_g_b' * [u v w]';
%     
%     pn_dot = p_dot(1); pe_dot = p_dot(2); pd_dot = p_dot(3);    % x y z
%     
%     % вращательная кинематика
%     tmp_matrix = [...
%                 1, -cos(phi)*tan(theta), sin(phi)*tan(theta);  ...
%                 0, cos(phi)/cos(theta),  -sin(phi)/cos(theta); ...
%                 0, sin(phi),             cos(phi)              ...
%               ];
%     euler_angles = tmp_matrix * [p q r]';
%     phi_dot = euler_angles(1); 
%     psi_dot = euler_angles(2); theta_dot =  euler_angles(3);
% %     psi_dot = 0; phi_dot = 0; theta_dot = 0;
%     
%     % поступательная динамика
% %     tmp_matrix = [ ...
% %                 r*v - q*w; ...
% %                 p*w - r*u; ...
% %                 q*u - p*v  ...
% %               ];
%     speed_dot = hat([u v w]')*[p q r]' + (1/MAV.mass) * F_b;
%     u_dot = speed_dot(1); v_dot = speed_dot(2); w_dot = speed_dot(3);
%     
%     % вращательная динамика
%     pqr_dot = MAV.J_inv * ( hat([p q r]) * MAV.J * [p q r]' + M_b );
%     p_dot = pqr_dot(1); q_dot = pqr_dot(2); r_dot = pqr_dot(3);
% 
% %         p_dot = 0; q_dot = 0; r_dot = 0; 
%     
%     sys = [ ...
%             pn_dot pe_dot pd_dot        ...
%             u_dot v_dot w_dot           ...
%             phi_dot theta_dot psi_dot   ...
%             p_dot q_dot r_dot           ...
%           ]';
% %     sys = zeros(12, 1);

% end mdlDerivatives

% function sys=mdlDerivativesMatrix(t,x,uu, MAV)
% %   Earth coordinates
%     p = x(1:3)
% %   speed in 'b' coordinates
%     V = x(4:6);
% %   Euler angles
%     euler = x(7:9);
% %   angular speeds
%     omega = x(10:12);
% %   inputs
%     F = uu(1:3)
%     M = uu(4:6);
%     
%     
%     % поступательная кинематика
%     p_dot = MAV.R_g_b(euler)' * V;
%     
%     % вращательная кинематика
%     tmp_matrix = [...
%                 1, -cos(phi)*tan(theta), sin(phi)*tan(theta);  ...
%                 0, cos(phi)/cos(theta),  -sin(phi)/cos(theta); ...
%                 0, sin(phi),             cos(phi)              ...
%               ];
%     euler_dot = tmp_matrix * omega;
%     
%     % поступательная динамика
% %     tmp_matrix = [ ...
% %                 r*v - q*w; ...
% %                 p*w - r*u; ...
% %                 q*u - p*v  ...
% %               ];
%     V_dot = hat(V)*omega + (1/MAV.mass) * F;
%     
%     % вращательная динамика
%     pqr_dot = MAV.J_inv * ( hat(omega) * MAV.J * omega + M );
%     sys = [
%         p_dot;
%         euler_dot;  % !!! поменять порядок углов
% 
%     ];
%     
%     sys = [ ...
%             pn_dot pe_dot pd_dot        ...
%             u_dot v_dot w_dot           ...
%             phi_dot theta_dot psi_dot   ...
%             p_dot q_dot r_dot           ...
%           ]';
% end mdlDerivativesMatrix

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x)
    y = [...
        x(1);
        x(2);
        x(3);
        x(4);
        x(5);
        x(6);
        x(7);
        x(8);
        x(9);
        x(10);
        x(11);
        x(12);
        ];
sys = y;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
