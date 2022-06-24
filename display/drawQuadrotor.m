function drawQuadrotor(uu)
    t        = uu(end);
%     FPS = 30;
%     persistent lastDrawTime
%     if isempty(lastDrawTime)
%         lastDrawTime = -inf;
%     end
%     if t - lastDrawTime < 1/FPS
%         return
%     end
%     lastDrawTime = t;
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
        pn, pe, pd, ...
        u, v, w, ...
        phi, theta, psi, ...
        p, q, r, ...
    ] = uu_cell{idx_map{1}};
    
    % target object
    target_idx = idx_map{6};
    x_target = uu_cell{target_idx(1)};
    y_target = uu_cell{target_idx(2)};

%     phi = deg2rad(phi);
%     theta = deg2rad(theta);
%     psi = deg2rad(psi);

%     % process inputs to function
%     pn       = uu(1);       % inertial North position     
%     pe       = uu(2);       % inertial East position
%     pd       = uu(3);           
%     u        = uu(4);       
%     v        = uu(5);       
%     w        = uu(6);       
%     phi      = uu(7);       % roll angle         
%     theta    = uu(8);       % pitch angle     
%     psi      = uu(9);       % yaw angle     
%     p        = uu(10);       % roll rate
%     q        = uu(11);       % pitch rate     
%     r        = uu(12);       % yaw rate  
%    
%     
% %     t        = uu(17 + 6 + 15);       % time
%     x_target = uu(17 + 6 + 15);
%     y_target = uu(17 + 6 + 15 + 1);
%     t        = uu(end);
%     FPS = 30;
%     persistent lastDrawTime
%     if isempty(lastDrawTime)
%         lastDrawTime = -inf;
%     end
%     if t - lastDrawTime < 1/FPS
%         return
%     end
%     lastDrawTime = t;

    % define persistent variables 
    persistent aircraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
%     x_size = 10;
%     y_size = 10;
%     z_size = 10;
        x_size = 1;
    y_size = 1;
    z_size = 1;
    
    need_follow_quad = 1;
    if need_follow_quad
        ctr = [pn pe pd]';
%         hold off;
%         ylim("auto")
%        axis([-z_size + ctr(3), 0.3 + ctr(3),-x_size + ctr(1), 0.3 + ctr(1), -0.3+ctr(2), y_size + ctr(2)]); 
       axis([ctr(3)-z_size/2, ctr(3) + z_size/2, ctr(1) - x_size/2, ctr(1) + x_size/2, ctr(2) - y_size/2, ctr(2) + y_size/2]);
%                hold on
    end
        
    % Init
    if t==0
        figure(1), clf
        [Vertices, Faces, facecolors] = defineQuadrotor;
        aircraft_handle = drawQuadrotorBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               x_target, y_target, ...
                                               [],'normal');
        title('Aircraft')
        xlabel('Z (x)')
        ylabel('X (y)')
        zlabel('Y (z)')
%         view(135,35)  % set the view angle for figure
        view(120,20)

%         ctr = [112 0 0]';
%         ylim("manual")
%         самолетная система координат
        [center, s_size] = Utils.getCenter();
        set(gcf, 'Position', [center(3) 100 center(3) center(4)*2-300]);
%         axis([-z_size + ctr(3), 0.3 + ctr(3),-x_size + ctr(1), 0.3 + ctr(1), -0.3+ctr(2), y_size + ctr(2)]);
%         hold on
%     ctr = [pn pe pd]';
%     axis([ctr(3)-z_size/2, ctr(3) + z_size/2, ctr(1) - x_size/2, ctr(1) + x_size/2, ctr(2) - y_size/2, ctr(2) + y_size/2]);
    
        
    % at every other time step, redraw base and rod
    else 
        
        drawQuadrotorBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           x_target, y_target, ...
                           aircraft_handle);
%        ax = gca;
%        ctr = [pn pe pd]';
%        ax.XLim = [ctr(3)-z_size/2, ctr(3) + z_size/2];
%        ax.YLim=[ctr(1)-x_size/2, ctr(1) + x_size/2];
%        ax.ZLim=[ctr(2)-y_size/2, ctr(2) + y_size/2];
%     ax.CameraTarget = [pn pe pd];
%     ax.CameraViewAngle = 15;
%     
    end

end

%=======================================================================
% drawAircraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawQuadrotorBody(V, F, patchcolors, ...
                                     pn, pe, pd, phi, theta, psi, ...
                                     xt, yt, ...
                                     handle, mode)

  V = rotate(V', phi, theta, psi)';  % rotate Aircraft

  V = translate(V', pn, pe, pd)';  % translate Aircraft
  % transform vertices from NED to XYZ (for matlab rendering)
  [Vt, Ft, Colort] = defineTarget();
  Vt = translate(Vt', xt, yt, 0)';
  V = [V; Vt];
  F = [F; Ft];
  patchcolors = [patchcolors; Colort];
  R = [...
      0, 1, 0; ...
      0, 0, 1; ...
      1, 0, 0; ...
  ];
  V = V*R;
  if isempty(handle)
      handle = patch('Vertices', V, 'Faces', F, ...
           'FaceVertexCData',patchcolors,...
            'FaceColor','flat',...
            'EraseMode', mode);
       grid on;
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow;
  end
end

function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;                  ...
          0, cos(phi), sin(phi);    ...
          0, -sin(phi), cos(phi)];
      
  R_pitch = [ ...
          cos(theta), sin(theta),  0; ...
          -sin(theta), cos(theta), 0; ...
          0,           0,          1  ...
          ];
  R_yaw = [ ...
            cos(psi), 0, -sin(psi); ...
            0,        1, 0;         ...
            sin(psi), 0, cos(psi)   ...
           ];
  R = R_roll*R_pitch*R_yaw;
  % rotate vertices
  XYZ = R'*XYZ;
end

% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat( [pn;pe;pd], 1, size(XYZ,2) );
end

% define aircraft vertices and faces
function [V,F,facecolors] = defineQuadrotor()

% линейные размеры, мм (для графики)
MAV.radius_l = 125;    % луч от центра до оси винта
MAV.radius_x = MAV.radius_l*cos(pi/4);  % проекция луча на ось
MAV.radius_a = 6;      % размер квадратного сечения луча
MAV.radius_a_x = MAV.radius_a*cos(45);
MAV.cockpit_side = 70;  % длина стороны кабины-куба
MAV.motor = [35 15];    % H, W параллелепипеда мотора
% Define the vertices (physical location of vertices
c = [ ...
    0 0 0; ...          % center of the quadrotor
    MAV.radius_x 0 -MAV.radius_x; ...   % left head motor
    MAV.radius_x 0 MAV.radius_x; ...   % right head motor
    -MAV.radius_x 0 MAV.radius_x; ...   % right back motor
    -MAV.radius_x 0  -MAV.radius_x; ...   % left back motor
];

% X - вправо, Y - вверх, Z - на нас
% по центру
kube = [ ...
   -1 1 -1;
   1 1 -1;
   1 1 1;
   -1 1 1;
   -1 -1 -1;
   1 -1 -1;
   1 -1 1;
   -1 -1 1;
];

cockpit = (kube + [0 1 0])* MAV.cockpit_side/2;

motor = (kube + [0 1 0]) .* [1 MAV.motor(1)/MAV.motor(2) 1] * MAV.motor(2)/2;

% left head radius
radius1 = [ ...
    0 0 0;
    MAV.radius_a_x 0 MAV.radius_a_x;
    MAV.radius_x+MAV.radius_a_x 0 -MAV.radius_x+MAV.radius_a_x;
    MAV.radius_x 0 -MAV.radius_x;
    0 MAV.radius_a_x 0;
    MAV.radius_a_x MAV.radius_a MAV.radius_a_x;
    MAV.radius_x+MAV.radius_a_x MAV.radius_a -MAV.radius_x+MAV.radius_a_x;
    MAV.radius_x MAV.radius_a -MAV.radius_x;
];

nose = [
    0 -1 -1;
    0 -1 1;
    0 1 1;
    0 1 -1;
    0.5 0 0;
];
nose = (nose .* [1 0.5 0.5] + [1 1 0]) * MAV.cockpit_side/2;

david_star = (kube + [0 -1 0]) .* [1 0.2 1] * 1e3;

V = [ ...
    cockpit;    % 1..8
    radius1;    % 9..16
    radius1 .* [1 1 -1]; % 17.
    radius1 .* [-1 1 -1];
    radius1 .* [-1 1 1];
    nose;
    motor + c(2,:);
    motor + c(3,:);
    motor + c(4,:);
    motor + c(5,:);
%     david_star;
];

V = 1e-3 * V;   % мм -> м

cockpit_faces = [...
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    4 3 7 8;
    2 6 7 3;
    1 4 8 5;
];

radius1_faces = [...
    1 2 3 4;
    5 6 7 8;
    2 3 7 6;
    1 4 8 5;
    1 2 6 5;
    3 4 8 7;
] + size(cockpit, 1);
nose_faces = [ ...
    1 2 3 4;
    1 2 5 1;
    2 3 5 1;
    3 4 5 3;
    4 1 5 1;
];

motor_faces = [...
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    4 3 7 8;
    2 6 7 3;
    1 4 8 5;
] + size(nose, 1) + size(radius1, 1)*4 + size(cockpit, 1);

F = [ ...
    cockpit_faces;
    radius1_faces;
    radius1_faces + size(radius1, 1);
    radius1_faces + size(radius1, 1)*2;
    radius1_faces + size(radius1, 1)*3;
    nose_faces + size(radius1, 1)*4 + size(cockpit, 1);
    motor_faces;
    motor_faces + size(motor, 1);
    motor_faces + size(motor, 1)*2;
    motor_faces + size(motor, 1)*3;
%     motor_faces + size(motor, 1)*4; % цель посадки
];
    
% define colors for each face 
  colors.grey = [0.5 0.5 0.5];
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];
  left_color = myred;
  right_color = mygreen;
  % задаем цвет на каждую грань
  facecolors = [...
    repmat(colors.grey, 6, 1);...
    repmat(left_color, 6, 1);...
    repmat(right_color, 6, 1);...
    repmat(right_color, 6, 1);...
    repmat(left_color, 6, 1);...
    repmat(mycyan, size(nose_faces, 1), 1);...
    repmat(left_color, 6, 1);...
    repmat(right_color, 6, 1);...
    repmat(right_color, 6, 1);...
    repmat(left_color, 6, 1);...
    ];
end

function [V,F,facecolors] = defineTarget()

% X - вправо, Y - вверх, Z - на нас
% по центру
kube = [ ...
   -1 1 -1;
   1 1 -1;
   1 1 1;
   -1 1 1;
   -1 -1 -1;
   1 -1 -1;
   1 -1 1;
   -1 -1 1;
];
david_star = (kube + [0 -1 0]) .* [0.2 0.2 0.2];

V = [ ...
    david_star;
];
kube_faces = [...
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    4 3 7 8;
    2 6 7 3;
    1 4 8 5;
];
F = [ ...
    kube_faces + 77;
];
% define colors for each face 
  colors.grey = [0.5 0.5 0.5];
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];
  % задаем цвет на каждую грань
  facecolors = [...
    repmat(colors.grey, 6, 1);...
 ];
end