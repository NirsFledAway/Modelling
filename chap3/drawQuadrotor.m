function drawQuadrotor(uu)
    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate  
    t        = uu(13);       % time

    % define persistent variables 
    persistent aircraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
        
    % Init
    if t==0
        figure(1), clf
        [Vertices, Faces, facecolors] = defineQuadrotor;
        aircraft_handle = drawQuadrotorBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Aircraft')
        xlabel('Z (x)')
        ylabel('X (y)')
        zlabel('Y (z)')
        view(135,35)  % set the view angle for figure
        x_size = 3;
        y_size = 3;
        z_size = 3;
%         самолетная система координат
        axis([-x_size,x_size,-z_size,z_size,-y_size,y_size]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawQuadrotorBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           aircraft_handle);
    end
end

%=======================================================================
% drawAircraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawQuadrotorBody(V, F, patchcolors, ...
                                     pn, pe, pd, phi, theta, psi, ...
                                     handle, mode)

  V = rotate(V', phi, theta, psi)';  % rotate Aircraft

  V = translate(V', pn, pe, pd)';  % translate Aircraft
  % transform vertices from NED to XYZ (for matlab rendering)
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
    drawnow
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