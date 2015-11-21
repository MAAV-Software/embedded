## Copyright (C) 2015 sajanptl
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {Function File} {@var{retval} =} spKFaccel (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: sajanptl <sheldon@Sheldon>
## Created: 2015-11-21

close all
clc 
clf
log = load('fixedEkf/LOG78.TXT');

% parsing the data
Time          = log(:,1);
Mode          = log(:,2);
Imu_AccX      = log(:,3);
Imu_AccY      = log(:,4);
Imu_AccZ      = log(:,5);
Imu_AngRateX  = log(:,6);
Imu_AngRateY  = log(:,7);
Imu_AngRateZ  = log(:,8);
Imu_MagX      = log(:,9);
Imu_MagY      = log(:,10);
Imu_MagZ      = log(:,11);
Imu_Rot       = log(:,12:20);
Px4_Xdot      = log(:,21);
Px4_Ydot      = log(:,22);
Px4_Qual      = log(:,23);
Lidar_Dist    = log(:,24);
Camera_X      = log(:,25);
Camera_Y      = log(:,26);
Camera_Yaw    = log(:,27);
Camera_T      = log(:,28);
Filter_X      = log(:,29);
Filter_Y      = log(:,30);
Filter_Z      = log(:,31);
Filter_Xdot   = log(:,32);
Filter_Ydot   = log(:,33);
Filter_Zdot   = log(:,34);
Filter_Roll   = log(:,35);
Filter_Pitch  = log(:,36);
Filter_Yaw    = log(:,37);
Val_P_X       = log(:,38);
Val_I_X       = log(:,39);
Val_D_X       = log(:,40);
Val_P_Y       = log(:,41);
Val_I_Y       = log(:,42);
Val_D_Y       = log(:,43);
Val_P_Z       = log(:,44);
Val_I_Z       = log(:,45);
Val_D_Z       = log(:,46);
Val_P_Yaw     = log(:,47);
Val_I_Yaw     = log(:,48);
Val_D_Yaw     = log(:,49);
Rate_P_X      = log(:,50);
Rate_I_X      = log(:,51);
Rate_D_X      = log(:,52);
Rate_P_Y      = log(:,53);
Rate_I_Y      = log(:,54);
Rate_D_Y      = log(:,55);
Rate_P_Z      = log(:,56);
Rate_I_Z      = log(:,57);
Rate_D_Z      = log(:,58);
Rate_P_Yaw    = log(:,59);
Rate_I_Yaw    = log(:,60);
Rate_D_Yaw    = log(:,61);
Setpt_X       = log(:,62);
Setpt_Y       = log(:,63);
Setpt_Z       = log(:,64);
Setpt_Yaw     = log(:,65);
Setpt_Xdot    = log(:,66);
Setpt_Ydot    = log(:,67);
Setpt_Zdot    = log(:,68);
PID_Ux        = log(:,69);
PID_Uy        = log(:,70);
PID_Uz        = log(:,71);
PID_Yawdot    = log(:,72);
Flag_Xval     = log(:,73);
Flag_Yval     = log(:,74);
Flag_Zval     = log(:,75);
Flag_Xrate    = log(:,76);
Flag_Yrate    = log(:,77);
Flag_Zrate    = log(:,78);
Flag_Yawl     = log(:,79);
Battery       = log(:,80);
DJI_Roll      = log(:,81);
DJI_Pitch     = log(:,82);
DJI_Yawdot    = log(:,83);
DJI_Fz        = log(:,84);
AtomFlag      = log(:,85);
DJI_Roll_RAW  = log(:,86);
DJI_Pitch_RAW = log(:,87);
DJI_Fz_RAW    = log(:,88);
DJI_YawRate_RAW = log(:,89);

PPM_1 = log(:, 90);
PPM_2 = log(:, 91);
PPM_3 = log(:, 92);
PPM_4 = log(:, 93);

roll = atan2(Imu_Rot(:,6), Imu_Rot(:,9));
pitch = asin(-Imu_Rot(:,3));
yaw = atan2(Imu_Rot(:,2), Imu_Rot(:,1));

%%
mass = 2.38;
gravity = 9.81;

x = zeros(9,1);           % [x y z xdot ydot zdot, xddot, yddot, zddot]
dx = zeros(9,1);          % [xdot ydot zdot xddot yddot zddot dAx dAy dAz] dAx, dAy, dAz will be 0 
                        % which is [x_4 x_5 x_6 Fx/m Fy/m Fz/m 0 0 0] where
                        % forces are obtained from decomposing u_1 = Fz_body
P = 0.1 * eye(9);
Q = zeros(9);
Q(1,1) = 0.1;
Q(2,2) = 0.1;
Q(3,3) = 0.9;
Q(4,4) = 0.01;
Q(5,5) = 0.01;
Q(6,6) = 0.01;
Q(7,7) = 0.1;
Q(8,8) = 0.1;
Q(9,9) = 0.1;

u = zeros(4,1);           % [Fz roll pitch yawRate] (to dji)

A = [[0 0 0 1 0 0 0 0 0];
     [0 0 0 0 1 0 0 0 0];
     [0 0 0 0 0 1 0 0 0];
     [0 0 0 0 0 0 1 0 0];
     [0 0 0 0 0 0 0 1 0];
     [0 0 0 0 0 0 0 0 1];
     [0 0 0 0 0 0 0 0 0];
     [0 0 0 0 0 0 0 0 0];
     [0 0 0 0 0 0 0 0 0]];
     
     
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
%xPred = [];
%P_pred = [];
%xCorr = [];
%P_Corr = [];

dx_vec = [];

px4_xdot = [];
px4_ydot = [];
Zdist = [];
%predTime = [];
%corrTime = [];
xFlt = [];
pFlt = [];

for i = 2:length(Time)
     Zdist = [Zdist, -Lidar_Dist(i) * cos(pitch(i)) * cos(roll(i))];
     px4_xdot = [px4_xdot, (Px4_Xdot(i) * cos(yaw(i))) + (Px4_Ydot(i) * sin(yaw(i)))];
     px4_ydot = [px4_ydot, -(Px4_Xdot(i) * sin(yaw(i))) + (Px4_Ydot(i) * sin(yaw(i)))];

  if ((Time(i) - Time(i-1)) < 0.015)
  %if (false)
     % fill in u, dt, dx
     u = [-DJI_Fz_RAW(i); DJI_Roll_RAW(i); DJI_Pitch_RAW(i); DJI_YawRate_RAW(i)];
     dt = Time(i) - Time(i-1);
     dx = [x(4);
           x(5); 
           x(6); 
           u(1) / mass * ( (cos(yaw(i))   * sin(pitch(i)) * cos(roll(i))) + (sin(yaw(i)) * sin(roll(i))) );
           u(1) / mass * ( (sin(yaw(i))   * sin(pitch(i)) * cos(roll(i))) - (cos(yaw(i)) * sin(roll(i))) );
           %u(1) / mass * (  cos(pitch(i)) * cos(roll(i)) )];
           0 / mass * cos(pitch(i)) * cos(roll(i))]; 
      dx_vec = [dx_vec, dx];
     
     % debug
     dx(6);
       
     % Run prediction step
     x = x + dt * dx;
     
     %debug
     x(6);
     
     P = P + (dt * ((A * P) + (P * A') + Q));
     %xPred = [xPred, x];
     %P_pred = [P_pred, [P(1, 1);
     %                   P(2, 2);
     %                   P(3, 3);
     %                   P(4, 4);
     %                   P(5, 5);
     %                   P(6, 6)]];
     %predTime = [predTime, Time(i)];     
  else
     % set up correction step 1
     y_1 = [x(3); x(4); x(5)];
     sensor_1 = [-Lidar_Dist(i) * cos(pitch(i)) * cos(roll(i));
                  (Px4_Xdot(i) * cos(yaw(i))) + (Px4_Ydot(i) * sin(yaw(i)));
                  -(Px4_Xdot(i) * sin(yaw(i))) + (Px4_Ydot(i) * sin(yaw(i)))];
     
              
     L = P * C_1' * inv(R_1 + (C_1 * P * C_1'));
     P = (eye(6) - (L * C_1)) * P;
     
     x = x + (L * (-y_1 + sensor_1));
     
     %debug
     sensor_1(1);
     x(6);
     
     %xCorr = [xCorr, x];
     %P_Corr = [P_Corr, [P(1, 1);
     %                   P(2, 2);
     %                   P(3, 3);
     %                   P(4, 4);
     %                   P(5, 5);
     %                   P(6, 6)]]; 
     %corrTime = [corrTime, Time(i)];    
  end
  
  xFlt = [xFlt, x];
  pFlt = [pFlt, [P(1, 1);
                 P(2, 2);
                 P(3, 3);
                 P(4, 4);
                 P(5, 5);
                 P(6, 6)]];
end


figure(1)
hold on;
%plot(corrTime, -xCorr(3,:), 'b');
%plot(predTime, -xPred(3,:), '--k');
plot(Time(2:end), -xFlt(3,:), 'b');
plot(Time(2:end), -Zdist, 'g');
plot(Time(2:end), -(xFlt(3,:) + 3 * sqrt(pFlt(3,:))), '--r');
plot(Time(2:end), -(xFlt(3,:) - 3 * sqrt(pFlt(3,:))), '--r');
%plot(corrTime, -(xCorr(3,:) + 3 * sqrt(P_Corr(3,:))), '--r');
%plot(corrTime, -(xCorr(3,:) - 3 * sqrt(P_Corr(3,:))), '--r');
xlabel('Time');
ylabel('Z');
hold off;

figure(2)
hold on;

%plot(corrTime, xCorr(4,:), 'b');
%plot(predTime, xPred(4,:), '--b');

plot(Time(2:end), xFlt(4,:), 'b');
plot(Time(2:end), px4_xdot, 'g');
plot(Time(2:end), (xFlt(4,:) + 3 * sqrt(pFlt(4,:))), '--r');
plot(Time(2:end), (xFlt(4,:) - 3 * sqrt(pFlt(4,:))), '--r');

%plot(corrTime, (xCorr(4,:) + 3 * sqrt(P_Corr(4,:))), '--r')
%plot(corrTime, (xCorr(4,:) - 3 * sqrt(P_Corr(4,:))), '--r');
%plot(Time(2:end), Setpt_Xdot(2:end), 'k');

xlabel('Time');
ylabel('Xdot');
hold off;

figure(3)
hold on;

%plot(corrTime, xCorr(5,:), 'b');
%plot(predTime, xPred(5,:), '--b');

plot(Time(2:end), xFlt(5,:), 'b');
plot(Time(2:end), px4_ydot, 'g');
plot(Time(2:end), (xFlt(5,:) + 3 * sqrt(pFlt(5,:))), '--r');
plot(Time(2:end), (xFlt(5,:) - 3 * sqrt(pFlt(5,:))), '--r');

%plot(corrTime, (xCorr(5,:) + 3 * sqrt(P_Corr(5,:))), '--r');
%plot(corrTime, (xCorr(5,:) - 3 * sqrt(P_Corr(5,:))), '--r');
%xlabel('Time');
%ylabel('Ydot');
%hold off;


figure(4)
hold on;
plot(Time(2:end), xFlt(1,:), 'b');
plot(Time(2:end), px4_xdot, 'g');
plot(Time(2:end), (xFlt(1,:) + 3 * sqrt(pFlt(1,:))), '--r');
plot(Time(2:end), (xFlt(1,:) - 3 * sqrt(pFlt(1,:))), '--r');
xlabel('Time');
ylabel('X');
hold off;

%figure(4)
%plot(Time(2:end), DJI_Fz_RAW(2:end), '-k');
%plot(Time(2:end), Time(2:end) - Time(1:end-1), '-k');
%figure(1)
%plot(Time(2:end), -xCorr(3,:), 'b', ...
%     Time(2:end), -xPred(3,:), '--b', ...
%     Time(2:end), -Zdist, 'g', ...
%     Time(2:end), -(xCorr(3,:) + 3 * sqrt(P_Corr(3,:))), '--r', ...
%     Time(2:end), -(xCorr(3,:) - 3 * sqrt(P_Corr(3,:))), '--r');
%xlabel('Time');
%ylabel('Z');

%figure(2)
%plot(Time(2:end), xCorr(4,:), 'b', ...
%     Time(2:end), xPred(4,:), '--b', ...
%     Time(2:end), px4_xdot, 'g', ...
%     Time(2:end), (xCorr(4,:) + 3 * sqrt(P_Corr(4,:))), '--r', ...
%     Time(2:end), (xCorr(4,:) - 3 * sqrt(P_Corr(4,:))), '--r', ...
%     Time(2:end), Setpt_Xdot(2:end), 'k');
%xlabel('Time');
%ylabel('Xdot');

%figure(3)
%plot(Time(2:end), xCorr(5,:), 'b', ...
%     Time(2:end), xPred(5,:), '--b', ...
%     Time(2:end), px4_ydot, 'g', ...
%     Time(2:end), (xCorr(5,:) + 3 * sqrt(P_Corr(5,:))), '--r', ...
%     Time(2:end), (xCorr(5,:) - 3 * sqrt(P_Corr(5,:))), '--r');
%xlabel('Time');
%ylabel('Ydot');

%      Time(2:end), px4_ydot, 'g', ...
%      Time(2:end), (xCorr(5,:) + 3 * sqrt(P_Corr(5,:))), '--r', ...
%      Time(2:end), (xCorr(5,:) - 3 * sqrt(P_Corr(5,:))), '--r');

%      Time(2:end), xPred(4,:), '--b', ...

%      Time(2:end), px4_xdot, 'g', ...
%      Time(2:end), (xCorr(4,:) + 3 * sqrt(P_Corr(4,:))), '--r', ...
%      Time(2:end), (xCorr(4,:) - 3 * sqrt(P_Corr(4,:))), '--r');
