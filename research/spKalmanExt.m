close all
clc 
clf

%%
mass = 2.38;
gravity = 9.81;

Lidar_Dist    = 1.5; %meters


Px4_Xdot      = 1;
Px4_Ydot      = 1; %m/s
DJI_Fz_RAW    = mass*gravity;


roll = 0;
pitch = 0;
yaw = 0;

dt = 0.01;




x = zeros(6,1);           % [x y z xdot ydot zdot]
dx = zeros(6,1);          % [xdot ydot zdot xddot yddot zddot] 
                        % which is [x_4 x_5 x_6 Fx/m Fy/m Fz/m] where
                        % forces are obtained from decomposing u_1 =
                        % Fz_body
P = 0.1 * eye(6);
Q = zeros(6);
Q(1,1) = 0.1;
Q(2,2) = 0.1;
Q(3,3) = 0.9;
Q(4,4) = 0.01;
Q(5,5) = 0.01;
Q(6,6) = 0.01;

u = zeros(4,1);           % [Fz roll pitch yawRate] (to dji)

A = [[0 0 0 1 0 0];
     [0 0 0 0 1 0];
     [0 0 0 0 0 1];
     [0 0 0 0 0 0];
     [0 0 0 0 0 0];
     [0 0 0 0 0 0]];
     
     
%sensor_1 = zeros(3,1);    % [z xdot ydot]
%y_1 = zeros(3,1);         % [x_3 x_4 x_5]
R_1 = zeros(3);  
R_1(1,1) = 0.04;
R_1(2,2) = 0.1;
R_1(3,3) = 0.1;

C_1 = [[0 0 1 0 0 0];   
       [0 0 0 1 0 0];   
       [0 0 0 0 1 0]];  

%sensor_2 = zeros(5,1);    % [x y z xdot ydot]
%y_2 = zeros(5,1);         % [x_1 x_2 x_3 x_4 x_5]
R_2 = zeros(5);
R_2(1,1) = 0.1;
R_2(2,2) = 0.1;
R_3(3,3) = 0.01;
R_4(4,4) = 0.1;
R_5(5,5) = 0.1;

C_2 = [[1 0 0 0 0 0];
       [0 1 0 0 0 0];
       [0 0 1 0 0 0];
       [0 0 0 1 0 0];
       [0 0 0 0 1 0]];


dx_vec = [];

px4_xdot = [];
px4_ydot = [];
Zdist = [];

xFlt = [];
pFlt = [];

disp('Initial P = ');
disp(P);
disp('Initial x = ');
disp(x);



 
userUpPred = input('Run prediction, or update Kalman? (p/u): ', 's');



Zdist = -Lidar_Dist * cos(pitch) * cos(roll);
px4_xdot = (Px4_Xdot * cos(yaw)) + (Px4_Ydot * sin(yaw));
px4_ydot = -(Px4_Xdot * sin(yaw)) + (Px4_Ydot * sin(yaw));


    
if (userUpPred == 'p')
   disp('RUN PREDICTION');
   u = -DJI_Fz_RAW;
   
   dx = [x(4);
         x(5); 
         x(6); 
         u / mass * ( (cos(yaw)   * sin(pitch) * cos(roll)) + (sin(yaw) * sin(roll)) );
         u / mass * ( (sin(yaw)   * sin(pitch) * cos(roll)) - (cos(yaw) * sin(roll)) );
         u / mass * (  cos(pitch) * cos(roll) )]; 
    dx_vec = [dx_vec, dx];
     
   % debug
   dx(6);
       
   % Run prediction step
   x = x + dt * dx;
   disp('X = ');
   disp(x);
     
   %debug
   x(6);
     
   P = P + (dt * ((A * P) + (P * A') + Q));
   disp('P = ');
   disp(P);
   P = sqrt(P);
   disp('sqrt(P = ');
   disp(P);
     
else
   disp('RUN UPDATE');
   y_1 = [x(3); x(4); x(5)];
   sensor_1 = [-Lidar_Dist * cos(pitch) * cos(roll);
                (Px4_Xdot * cos(yaw)) + (Px4_Ydot * sin(yaw));
                -(Px4_Xdot * sin(yaw)) + (Px4_Ydot * cos(yaw))];
     
              
   L = P * C_1' * inv(R_1 + (C_1 * P * C_1'));
   disp('L = ');
   disp(L);
   P = (eye(6) - (L * C_1)) * P;
   disp('P = ');
   disp(P);
   x = x + (L * (-y_1 + sensor_1));
   disp('X = ');
   disp(x);
     
   %debug
   sensor_1(1);
   x(6);
     
    
end
  
xFlt = [xFlt, x];
pFlt = [pFlt, [P(1, 1);
                 P(2, 2);
                 P(3, 3);
                 P(4, 4);
                 P(5, 5);
                 P(6, 6)]];
%disp('xFlt = ');
%disp(xFlt);
%disp('pFlt = ');
%disp(pFlt);
