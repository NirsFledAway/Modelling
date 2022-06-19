function plotMAVStateVariables(uu)
    FPS = 30;
    t = uu(end);
    persistent lastDrawTime
    if isempty(lastDrawTime)
        lastDrawTime = -inf;
    end
    if t - lastDrawTime < 1/FPS
        return
    end
    lastDrawTime = t;

    idx_map = Utils.gen_idx([...
        12, ... % x
        4, ...  % u
        2, ...  % theta, theta_c
        9, ...  % desired
        15, ... % corrected x
        9 ...   % target state
    ]);
    
    uu_cell = num2cell(uu);
    
    [
        pn, pe, h, ...
        u, v, w, ...
        phi, theta, psi, ...
        p, q, r, ...
    ] = uu_cell{idx_map{1}};
    
    [ u1, u_x, u_y, u2 ] = uu_cell{idx_map{2}};
    
    [theta_c, theta_c_dot] = uu_cell{idx_map{3}};
    
    corrected_state_idx = idx_map{5};
    [v_x_g, v_y_g] = uu_cell{corrected_state_idx(4:5)};
    
    % desired values
    target_idx = idx_map{4};
    x_target = uu_cell{target_idx(1)};
    y_target = uu_cell{target_idx(2)};
    v_x_target = uu_cell{target_idx(4)};
    v_y_target = uu_cell{target_idx(5)};

    angles_deg = rad2deg([phi theta psi p q r theta_c theta_c_dot]);
    angles_deg_cell = num2cell(angles_deg);
    [...
        phi, theta, psi, ...
        p, q, r, ...
        theta_c, theta_c_dot, ...
    ] = angles_deg_cell{:};
    

    % TODO: Vz не отображается
  % init schema
  variables_list = [
    create_graph_2params(pn, x_target, 'x', []);  % 1
    create_graph_2params(pe, y_target, 'y', []);  % 2
    create_graph_params(h, 'z', []);    % 3
    create_graph_3params(v_x_g, v_x_target, u, 'v_x', []);    % 4
    create_graph_3params(v_y_g, v_y_target, v, 'v_y', []);    % 5
    create_graph_params(w, 'v_z', []);    % 6
    create_graph_params(phi, '\phi', []);    % 7
    create_graph_params(psi, '\psi', []);    % 8
    create_graph_2params(theta, theta_c, '\vartheta', []);    % 9
    create_graph_params(p, '\omega_x', []);    % 10
    create_graph_params(q, '\omega_y', []);    % 11
    create_graph_2params(r, theta_c_dot, '\omega_z', []);    % 12
    create_graph_params(u1, 'u_1', []);          % 13
    create_graph_params(u2, 'u_2', []);          % 14
  ];
%   linear = [1 2 3; 4 5 6]';
%   angular = [7 8 9; 10 11 12]';
  linear = [1 2; 4 5]';
  angular = [9; 12]';
  % map = [1 2 3; 4 5 6]';
  map = [linear; angular; [13 14]];

  % init params
  persistent handles;

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
    [center, s_size] = Utils.getCenter();
    set(gcf, 'Position', [0 0 center(3) center(4)*2]);
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
              if ~isnan(param.val_estimated)
                  handle = graph_y_yhat_yd(t, param);
              else
                handle = graph_y_yd(t, param);
              end
          else
              handle = graph_y(t, param);
          end
          idx = map(i,j);
          handles(idx).v = handle;
          if i == 1 && j == 2
              Lgnd = legend('show');
            Lgnd.Position(1) = 0.9;
            %     Lgnd.Position(2) = 0.4;
            legend('basic', 'desired', 'local');    
            end
        else
          if ~isnan(param.val_desired)
              if ~isnan(param.val_estimated)
                handle = graph_y_yhat_yd(t, param);
              else
                handle = graph_y_yd(t, param);
              end
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
    
function pack = create_graph_3params(val, des_val, est_val, name, handle)
    pack = create_graph_2params(val, des_val, name, handle);
    pack.val_estimated = est_val;


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
    handle(2)    = plot(t,yd,'r');
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
  t;

  % if isempty(handle) || length(handle) < 3,
  if t == 0
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yd,'r');
    handle(3)   = plot(t,yhat, 'g');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    disp('set')
    get(handle(1),'Xdata');
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yhat]);     
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

% function vals = gen_idx(sizes)
%     vals = cell(length(sizes), 1);
%     prevIdx = 1;
%     for i = 1:length(sizes)
%         nextIdx = prevIdx + sizes(i);
%         vals{i} = (prevIdx : nextIdx - 1);
%         prevIdx = nextIdx;
%     end


