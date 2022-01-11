function plotMAVStateVariables(uu)
%
% modified 12/11/2009 - RB

    % process inputs to function
    % quadrotor model
    pn          = uu(1);             % North position (meters)
    pe          = uu(2);             % East position (meters)
    h           = uu(3);             % altitude (meters)

    u           = uu(4);             % body velocity along x-axis (meters/s)
    v           = uu(5);             % body velocity along y-axis (meters/s)
    w           = uu(6);             % body velocity along z-axis (meters/s)

    phi         = 180/pi*uu(7);      % roll angle (degrees)   
    theta       = 180/pi*uu(8);      % pitch angle (degrees)
    psi         = 180/pi*uu(9);      % yaw angle (degrees)

    p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
    q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
    r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)

    % external inputs
%     Va          = uu(13);            % airspeed (m/s)
%     alpha       = 180/pi*uu(14);     % angle of attack (degrees)
%     beta        = 180/pi*uu(15);     % side slip angle (degrees)
%     wn          = uu(16);            % wind in the North direction
%     we          = uu(17);            % wind in the East direction
%     wd          = uu(18);            % wind in the Down direction
%     pn_c        = uu(19);            % commanded North position (meters)
%     pe_c        = uu(20);            % commanded East position (meters)
%     h_c         = uu(21);            % commanded altitude (meters)
%     Va_c        = uu(22);            % commanded airspeed (meters/s)
%     alpha_c     = 180/pi*uu(23);     % commanded angle of attack (degrees)
%     beta_c      = 180/pi*uu(24);     % commanded side slip angle (degrees)
%     phi_c       = 180/pi*uu(25);     % commanded roll angle (degrees)   
%     theta_c     = 180/pi*uu(26);     % commanded pitch angle (degrees)
%     chi_c       = 180/pi*uu(27);     % commanded course (degrees)
%     p_c         = 180/pi*uu(28);     % commanded body angular rate along x-axis (degrees/s)
%     q_c         = 180/pi*uu(29);     % commanded body angular rate along y-axis (degrees/s)
%     r_c         = 180/pi*uu(30);     % commanded body angular rate along z-axis (degrees/s)
%     pn_hat      = uu(31);            % estimated North position (meters)
%     pe_hat      = uu(32);            % estimated East position (meters)
%     h_hat       = uu(33);            % estimated altitude (meters)
%     Va_hat      = uu(34);            % estimated airspeed (meters/s)
%     alpha_hat   = 180/pi*uu(35);     % estimated angle of attack (degrees)
%     beta_hat    = 180/pi*uu(36);     % estimated side slip angle (degrees)
%     phi_hat     = 180/pi*uu(37);     % estimated roll angle (degrees)   
%     theta_hat   = 180/pi*uu(38);     % estimated pitch angle (degrees)
%     chi_hat     = 180/pi*uu(39);     % estimated course (degrees)
%     p_hat       = 180/pi*uu(40);     % estimated body angular rate along x-axis (degrees/s)
%     q_hat       = 180/pi*uu(41);     % estimated body angular rate along y-axis (degrees/s)
%     r_hat       = 180/pi*uu(42);     % estimated body angular rate along z-axis (degrees/s)
% %    Vg_hat      = uu(43);            % estimated groundspeed
% %    wn_hat      = uu(44);            % estimated North wind
% %    we_hat      = uu(45);            % estimated East wind
% %    psi_hat     = 180/pi*uu(46);     % estimated heading
% %    bx_hat      = uu(47);            % estimated x-gyro bias
% %    by_hat      = uu(48);            % estimated y-gyro bias
% %    bz_hat      = uu(49);            % estimated z-gyro bias
%     delta_e     = 180/pi*uu(50);     % elevator angle (degrees)
%     delta_a     = 180/pi*uu(51);     % aileron angle (degrees)
%     delta_r     = 180/pi*uu(52);     % rudder angle (degrees)
%     delta_t     = uu(53);            % throttle setting (unitless)
  u1 = uu(13);
  u2 = uu(14);
  theta_c = rad2deg(uu(15));
    t           = uu(16);            % simulation time

    FPS = 100;
    persistent lastDrawTime
    if isempty(lastDrawTime)
        lastDrawTime = -inf;
    end
    if t - lastDrawTime < 1/FPS
        return
    end
    lastDrawTime = t;
    

    
    % compute course angle
    % chi = 180/pi*atan2(Va*sin(psi)+we, Va*cos(psi)+wn);

    % TODO: Vz не отображается
  % init schema
  variables_list = [
    create_graph_params(pn, 'x', []);  % 1
    create_graph_params(pe, 'y', []);  % 2
    create_graph_params(h, 'z', []);    % 3
    create_graph_params(u, 'v_x', []);    % 4
    create_graph_params(v, 'v_y', []);    % 5
    create_graph_params(w, 'v_z', []);    % 6
    create_graph_params(phi, '\phi', []);    % 7
    create_graph_params(psi, '\psi', []);    % 8
    create_graph_2params(theta, theta_c, '\vartheta', []);    % 9
    create_graph_params(p, '\omega_x', []);    % 10
    create_graph_params(q, '\omega_y', []);    % 11
    create_graph_params(r, '\omega_z', []);    % 12
    create_graph_params(u1, 'u_1', []);          % 13
    create_graph_params(u2, 'u_2', []);          % 14
  ];
  linear = [1 2 3; 4 5 6]';
  angular = [7 8 9; 10 11 12]';
  % map = [1 2 3; 4 5 6]';
  map = [linear; angular; [13 14]];

  % init params
  persistent handles

  [variables, handles] = init_variables(variables_list, map, handles);

  handles = draw(variables, map, handles, t);
  
function [variables, handles] = init_variables(list, map, handles)
  if isempty(handles)
    handles = repmat(struct('v', []), size(list,1), 1);
  end
  variables = repmat(create_graph_params(0, '', []), size(map, 1), size(map, 2));
  for i=1:1:size(map, 1)
    for j=1:1:size(map, 2)
      param_idx = map(i,j);
      if param_idx ~= 0
        variables(i, j) = list(param_idx);
        variables(i,j).handle = handles(param_idx).v;
      end
    end
  end

function handles = draw(variables, map, handles, t)
  if t == 0
    figure(2), clf
  end

  curr = 0;
  for i=1:size(map, 1)
    for j=1:size(map, 2)
        curr = curr + 1;
        param = variables(i,j);
        if param.name == ""
            continue
        end
        if t == 0
          subplot(size(variables, 1), size(variables, 2), curr);
          hold on;
          if ~isnan(param.val_desired)
              handle = graph_y_yd(t, param);
          else
              handle = graph_y(t, param);
          end
          idx = map(i,j);
          handles(idx).v = handle;
        else
          if ~isnan(param.val_desired)
              graph_y_yd(t, param);
          else
              graph_y(t, param);
          end
        end
    end
  end

function pack = create_graph_params(val, name, handle)
  pack.val = val;
  pack.val_estimated = nan;
  pack.val_desired = nan;
  pack.name = name;
  pack.handle = handle;
  
function pack = create_graph_2params(val, des_val, name, handle)
    pack = create_graph_params(val, name, handle);
    pack.val_desired = des_val;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, params)
  y = params.val;
  lab = params.name;
  handle = params.handle;
  if isempty(handle) || isempty(handle.ColorMode),
    handle    = plot(t,y,'b');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation', 0);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, params)
  y = params.val;
  yd = params.val_desired;
  lab = params.name;
  handle = params.handle;
  if isempty(handle),
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'g');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, params)
  y = params.val;
  yhat = params.val_estimated;
  yd = params.val_desired;
  lab = params.name;
  handle = params.handle;
  t

  % if isempty(handle) || length(handle) < 3,
  if t == 0
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    disp('set')
    handle
    get(handle(1),'Xdata')
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
    %drawnow
  end

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low,
      out = low;
  elseif in > high,
      out = high;
  else
      out = in;
  end

% end sat  


