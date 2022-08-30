function plotMAVStateVariables(uu)
    FPS = 5;
    PPS = 30;   % points per second
    t = uu(end);
    persistent lastWorkedTime
    if isempty(lastWorkedTime)
        lastWorkedTime = [-inf; -inf];
    end
    if t - lastWorkedTime(1) < 1/PPS
        return
    end
    lastWorkedTime(1) = t;
    need_draw = 0;
    if t - lastWorkedTime(2) >= 1/FPS
        need_draw = 1;
        lastWorkedTime(2) = t;
%         profile on
    end


    idx_map = Utils.gen_idx([...
        12, ... % x
        4, ...  % u
        6, ...  % internal commands - euler angles and dots
        9, ...  % desired
        15, ... % corrected x
        9 ...   % target state
    ]);
    
    uu_cell = num2cell(uu);
    
    [
        x, y, z, ...
        v_x, v_y, v_z, ...
        phi, theta, psi, ...
        w_x, w_y, w_z, ...
    ] = uu_cell{idx_map{1}};
    
    [ u1, u_x, u_y, u_z ] = uu_cell{idx_map{2}};
    
    [phi_c, phi_c_dot, psi_c, psi_c_dot, theta_c, theta_c_dot] = uu_cell{idx_map{3}};
    
    corrected_state_idx = idx_map{5};
    [v_x_g, v_y_g, v_z_g] = uu_cell{corrected_state_idx(4:6)};
    
    % desired values
    target_idx = idx_map{4};
    x_target = uu_cell{target_idx(1)};
    y_target = uu_cell{target_idx(2)};
    z_target = uu_cell{target_idx(3)};
    v_x_target = uu_cell{target_idx(4)};
    v_y_target = uu_cell{target_idx(5)};
    v_z_target = uu_cell{target_idx(6)};

    angles_deg = rad2deg([phi theta psi w_x w_y w_z theta_c theta_c_dot]);
    angles_deg_cell = num2cell(angles_deg);
    [...
        phi, theta, psi, ...
        w_x, w_y, w_z, ...
        theta_c, theta_c_dot, ...
    ] = angles_deg_cell{:};

    
    g = 9.81;
    variables = create_graph_params({
        {'x', x, x_target};  % 1
        {'y', y, y_target};  % 2
        {'z', z, z_target, Inf};    % 3
        {'v_x', v_x_g, v_x_target, v_x};    % 4
        {'v_y', v_y_g, v_y_target, v_y};    % 5
        {'v_z', v_z_g, v_z_target, v_z};    % 6
        {'\phi', phi, phi_c};    % 7
        {'\psi', psi, psi_c};    % 8
        {'\vartheta', theta, theta_c};    % 9
        {'\omega_x', w_x, phi_c_dot};    % 10
        {'\omega_y', w_y, psi_c_dot};    % 11
        {'\omega_z', w_z, theta_c_dot};    % 12
        {'u_1', u1};          % 13
        {'u_x', u_x};          % 14
        {'u_y', u_y};          % 15
        {'u_z', u_z};          % 16
    %     {'v_d_dot', -g*sin(theta), w_z*v_y - w_y*v_z -g*sin(theta)};          % 15
    %     {v_z_dot coriolis', w_x*v_z - w_z*v_x, w_y*v_x - w_x*v_y};          % 16
    %     {'v_d_dot', -g*sin(theta), w_z*v_y - w_y*v_z -g*sin(theta)};          % 17
    %     {'w_x, phi_dot', w_x, w_x - cos(phi)*tan(theta)*w_y + sin(phi)*tan(theta)*w_z};          % 18
    %     {'w_x, psi_dot', w_y, cos(phi)*(1/cos(theta))*w_y - sin(phi)*(1/cos(theta))*w_z};          % 19
    %     {'w_z, theta_dot', w_z, sin(phi)*w_y + cos(phi)*w_z};          % 21
    });
    linear = [1 2 3; 4 5 6];
    angular = [7 8 9; 10 11 12];
    % linear = [1 2; 4 5]';
    % angular = [9; 12]';
    %   map = [1 2 3; 4 5 6]';
    map = [linear; angular; [13 0 0; 14 15 16]];
    %   map = [map(:, 1), map(:, 2), [15 18; 16 19; 17 20; 1 1]];

    handle_data(variables, map, t, need_draw);

function handle_data(variables, map, t, need_draw)
    persistent handles iteration storage y_range;
    if isempty(iteration)
      iteration = 0;
    end
    iteration = iteration + 1;
    
    [variables, handles] = init_variables(variables, handles);
    [storage, y_range] = store_points(storage, y_range, variables, t, iteration);
    
    if need_draw == 1
%         fA = parfeval(backgroundPool, @draw, variables, map, handles, storage, y_range, iteration);
        handles = draw(variables, map, handles, storage, y_range, iteration);
%         profile viewer
%         a = 1
    end
    
    
function [storage, y_range] = store_points(storage, y_range, variables, t, iteration)
    var_n = size(variables, 1);
    if isempty(storage)
        storage = cell(var_n + 1, 1);
    end
    if size(storage{1}, 2) < iteration
        for i = 1:var_n
            storage{i} = increaseMat(storage{i}, 2, NaN(length(variables{i}.values)));
        end
        storage{end} = increaseMat(storage{end}, 2, NaN(1,1));
    end
    storage{end}(iteration) = t;
    
    if isempty(y_range)
        y_range = Inf(var_n, 2) .* [1 -1];
    end
    
    for paramIndex = 1:var_n
        param = variables{paramIndex};
        storage{paramIndex}(:, iteration) = param.values';
        y_range(paramIndex, :) = [
            min(y_range(paramIndex, 1), min(param.values))
            max(y_range(paramIndex, 2), max(param.values))
        ];
    end

function handles = draw(variables, map, handles, storage, y_range, iteration)
    if iteration == 1
        figure(2), clf
        [center, s_size] = Utils.getCenter();
        set(gcf, 'Position', [0 0 center(3) center(4)*2-100]);
    end

    plotIndex = 0;
    for i=1:size(map, 1)
        for j=1:size(map, 2)
            plotIndex = plotIndex + 1;
            paramIndex = map(i, j);
            if paramIndex == 0
                % dont need to handle at all
                if iteration == 1
                    subplot(size(map, 1), size(map, 2), plotIndex);
                    hold on;
                    plot([0],[0]);
                end
                continue
            end
            param = variables{paramIndex};
            if isempty(handles(paramIndex).v)
                subplot(size(map, 1), size(map, 2), plotIndex);
                hold on;
                handle = graph([], param.name, storage{paramIndex}(:, 1:iteration), ...
                    storage{end}(1:iteration), y_range(paramIndex, :));
                handles(paramIndex).v = handle;
                if i == 1 && j == 2
                  Lgnd = legend('show');
                Lgnd.Position(1) = 0.9;
                legend('basic', 'desired', 'bound', 'error');    
                end
            else
                graph(param.handle, param.name, storage{paramIndex}(:, 1:iteration), ...
                    storage{end}(1:iteration), y_range(paramIndex, :));
            end
        end
    end

function handle = graph(handle, name, storage, time, y_range)
    t = time(end);
    values = storage(:, end);
    colors = ['b'; 'r'; 'g'; 'm'];
    if isempty(handle)
        for i = 1:length(values)
           handle(i) = plot(time, storage(i, :), colors(i));
        end
        
        set(handle(1), 'XLimInclude', 'off', 'YLimInclude', 'off');
        axes = get(handle(1), 'Parent');
        set(axes, 'XLimMode', 'manual', 'YLimMode', 'manual', ...
            'UserData', y_range, ...
            'XLim', [0 1], ...
            'YLim', get_y_limits(y_range, 0.05));
        set(get(gca, 'YLabel'), 'Rotation', 0.0);
        ylabel(name)
    else
        axes = get(handle(1), 'Parent');
        y_lim = get_y_limits(y_range, 0.05);
        set(axes, 'YLim', y_lim, 'XLim', [0, max(t, 1)], 'UserData', y_range);
        
        t_data = time;
        for i = 1:length(values)
            y_data = storage(i, :);
            set(handle(i), 'Xdata', t_data, 'Ydata', y_data);
        end
        handle = "fuck";
    end
% end graph

function [list, handles] = init_variables(list, handles)
  if isempty(handles)
    handles = repmat(struct('v', []), size(list,1), 1);
  end
  for i = 1:length(list)
     list{i}.handle = handles(i).v; 
  end

function packs = create_graph_params(template)
    packs = cell(size(template, 1), 1);
    for param_idx = 1:size(template, 1)
        param = template{param_idx};
        packs{param_idx} = struct('handle', [], 'name', param{1}, 'values', [param{2:end}]);
    end

function lims = get_y_limits(data_range, offset_factor)
    if data_range(1) == data_range(2) && data_range(1) == 0
       data_range(1) = -1e-20;
       data_range(2) = 1e-20;
    end
    % y_min - 5%; y_max + 5%
    lims = data_range + offset_factor*[-abs(data_range(1)), abs(data_range(2))];

function mat = increaseMat(src, factor, default)
    if size(src) == [0, 0]
       src = default; 
    end
    mat = [src, inf*ones(size(src, 1), max(size(src, 2)*(factor-1), 30))];

