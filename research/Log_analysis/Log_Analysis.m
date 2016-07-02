close all
clc
clf
log = load('2016/q2/LOG14.TXT');

% parsing the datas
Time          = log(2:end,1);
Mode          = log(2:end,2);
Imu_AccX      = log(2:end,3);
Imu_AccY      = log(2:end,4);
Imu_AccZ      = log(2:end,5);
Imu_AngRateX  = log(2:end,6);
Imu_AngRateY  = log(2:end,7);
Imu_AngRateZ  = log(2:end,8);
Imu_MagX      = log(2:end,9);
Imu_MagY      = log(2:end,10);
Imu_MagZ      = log(2:end,11);
Imu_Rot       = log(2:end,12:20);
Px4_Xdot      = log(2:end,21);
Px4_Ydot      = log(2:end,22);
Px4_Qual      = log(2:end,23);
Lidar_Dist    = log(2:end,24);
Camera_X      = log(2:end,25);
Camera_Y      = log(2:end,26);
Camera_Yaw    = log(2:end,27);
Camera_T      = log(2:end,28);
Filter_X      = log(2:end,29);
Filter_Y      = log(2:end,30);
Filter_Z      = log(2:end,31);
Filter_Xdot   = log(2:end,32);
Filter_Ydot   = log(2:end,33);
Filter_Zdot   = log(2:end,34);
Filter_Roll   = log(2:end,35);
Filter_Pitch  = log(2:end,36);
Imu_Comp_Yaw  = log(2:end,37);
Val_P_X       = log(2:end,38);
Val_I_X       = log(2:end,39);
Val_D_X       = log(2:end,40);
Val_P_Y       = log(2:end,41);
Val_I_Y       = log(2:end,42);
Val_D_Y       = log(2:end,43);
Val_P_Z       = log(2:end,44);
Val_I_Z       = log(2:end,45);
Val_D_Z       = log(2:end,46);
Val_P_Yaw     = log(2:end,47);
Val_I_Yaw     = log(2:end,48);
Val_D_Yaw     = log(2:end,49);
Rate_P_X      = log(2:end,50);
Rate_I_X      = log(2:end,51);
Rate_D_X      = log(2:end,52);
Rate_P_Y      = log(2:end,53);
Rate_I_Y      = log(2:end,54);
Rate_D_Y      = log(2:end,55);
Rate_P_Z      = log(2:end,56);
Rate_I_Z      = log(2:end,57);
Rate_D_Z      = log(2:end,58);
Rate_P_Yaw    = log(2:end,59);
Rate_I_Yaw    = log(2:end,60);
Rate_D_Yaw    = log(2:end,61);
Setpt_X       = log(2:end,62);
Setpt_Y       = log(2:end,63);
Setpt_Z       = log(2:end,64);
Setpt_Yaw     = log(2:end,65);
Setpt_Xdot    = log(2:end,66);
Setpt_Ydot    = log(2:end,67);
Setpt_Zdot    = log(2:end,68);
PID_Ux        = log(2:end,69);
PID_Uy        = log(2:end,70);
PID_Uz        = log(2:end,71);
PID_Yawdot    = log(2:end,72);
Flag_Xval     = log(2:end,73);
Flag_Yval     = log(2:end,74);
Flag_Zval     = log(2:end,75);
Flag_Xrate    = log(2:end,76);
Flag_Yrate    = log(2:end,77);
Flag_Zrate    = log(2:end,78);
Flag_Yawl     = log(2:end,79);
Battery       = log(2:end,80);
DJI_Roll      = log(2:end,81);
DJI_Pitch     = log(2:end,82);
DJI_Yawdot    = log(2:end,83);
DJI_Fz        = log(2:end,84);
AtomFlag          = log(2:end,85);
DJI_Roll_RAW      = log(2:end,86);
DJI_Pitch_RAW     = log(2:end,87);
DJI_Fz_RAW    = log(2:end,88);
DJI_YawRate_RAW        = log(2:end,89);

PPM_1 = log(2:end, 90);
PPM_2 = log(2:end, 91);
PPM_3 = log(2:end, 92);
PPM_4 = log(2:end, 93);

msTime = log(2:end, 94);
lidarTime = log(2:end, 95);
px4Time = log(2:end, 96);
imuTime = log(2:end, 97);
poseTime = log(2:end, 98);
refYaw = log(2:end, 99);
RCinError = log(2:end, 100);

clf

disc_Zdot = Filter_Zdot;
%%
for i = 2:length(Time)
    disc_Zdot(i) = (Lidar_Dist(i) - Lidar_Dist(i - 1)) / (Time(i) - Time(i-1));
end

%% plot
% figure 1 values
figure(1)
subplot(221)
plot(Time, Filter_X, 'b', Time, Setpt_X, 'g')
xlabel('Time')
ylabel('Value X')

subplot(222)
plot(Time, Filter_Y, 'b', Time, Setpt_Y, 'g')
xlabel('Time')
ylabel('Value Y')

subplot(223)
plot(Time, Lidar_Dist, 'b', Time, Setpt_Z, 'g')
xlabel('Time')
ylabel('Value Z')

subplot(224)
plot(Time, Imu_Comp_Yaw, 'b', Time, Setpt_Yaw, 'g')
xlabel('Time')
ylabel('Value Yaw')



% figure 2 rates
figure(2)
subplot(221)
plot(Time, Filter_Xdot, 'b', Time, Setpt_Xdot, 'g')
xlabel('Time')
ylabel('Rate X')

subplot(222)
xlabel('Time')
ylabel('Rate Y')
plot(Time, Filter_Ydot, 'b', Time, Setpt_Ydot, 'g')


subplot(223)
% plot(Time, Filter_Zdot, 'b', Time, Setpt_Zdot, 'g')
plot(Time, disc_Zdot, 'b', Time, Setpt_Zdot, 'g')
xlabel('Time')
ylabel('Rate Z')

% figure 3 DJI
figure(3)
title('RC Pilot Input (R, P, Y, T)');
subplot(221)
plot(Time(:), DJI_Roll(:), 'b')
xlabel('Time');
ylabel('DJI Roll PWM');
subplot(222)
plot(Time(:), DJI_Pitch(:), 'b')
xlabel('Time');
ylabel('DJI Pitch PWM');
subplot(223)
plot(Time(:), DJI_Yawdot(:), 'b')
xlabel('Time');
ylabel('DJI Yaw Dot PWM');
subplot(224)
plot(Time(:), DJI_Fz(:), '*b')
xlabel('Time');
ylabel('DJI Fz PWM');

%
%% figure 4 others
figure(4)
subplot(221)
xlabel('Time')
ylabel('Battery')
plot(Time, Battery, 'b')


%% figure 5 DJI RAW
figure(5)
subplot(221)
xlabel('Time')
ylabel('DJI Roll Raw')
plot(Time, DJI_Roll_RAW, 'b')
subplot(222)
xlabel('Time')
ylabel('DJI Pitch Raw')
plot(Time, DJI_Pitch_RAW, 'b')
subplot(224)
xlabel('Time')
ylabel('DJI Yaw Dot Raw')
plot(Time, DJI_YawRate_RAW, 'b')
subplot(223)
plot(Time, DJI_Fz_RAW, 'b')
xlabel('Time')
ylabel('DJI Fz Raw')

figure(6)
subplot(221)
plot(Time, PID_Uz, 'b')
xlabel('time')
ylabel('pid uz')


%figure(7)
%
%plot(Time, Rate_P_Z, 'b')

% figure 6 PID 
figure(80)
subplot(231)
plot(Time, Val_P_Z, 'b')
xlabel('Time')
ylabel('VAL P')
grid on

subplot(232)
plot(Time, Val_I_Z, 'b')
xlabel('Time')
ylabel('VAL I')
grid on

subplot(233)
plot(Time, Val_D_Z, 'b')
xlabel('Time')
ylabel('VAL D')
grid on

subplot(234)
plot(Time, Rate_P_Z, 'b')
xlabel('Time')
ylabel('Rate P')
grid on

subplot(235)
plot(Time, Rate_I_Z, 'b')
xlabel('Time')
ylabel('Rate I')
grid on

subplot(236)
plot(Time, Rate_D_Z, 'b')
xlabel('Time')
ylabel('Rate D')
grid on

%% 
figure(20)
subplot(221)
plot(Time, Px4_Xdot, 'b')
xlabel('Time')
ylabel('Px4 X vel')
grid on

subplot(222)
plot(Time, Px4_Ydot, 'b')
xlabel('Time')
ylabel('Px4 Y vel')
grid on

subplot(223)
plot(Time, Px4_Qual, 'b')
xlabel('Time')
ylabel('Qual')
grid on

figure(21)
plot(Time, Mode, 'b');
ylabel('Mode')
grid on

%{
figure(221)
plot(Time, PPM_1, 'b');
ylabel('Chan 1 PPM')

figure(222)
plot(Time, PPM_2, 'b');
ylabel('Chan 2 PPM')

figure(223)
plot(Time, PPM_3, 'b');
ylabel('Chan 3 PPM')

figure(224)
plot(Time, PPM_4, 'b');
ylabel('Chan 4 PPM')
%}

figure(124)
plot(Time, PPM_1, Time, PPM_2, Time, PPM_3, Time, PPM_4);
title('PPM Values')

figure(23)
subplot(221)
plot(Time, atan2(Imu_Rot(:, 6), Imu_Rot(:, 9)), 'b')
ylabel('Roll')

subplot(222)
plot(Time, asin(-Imu_Rot(:, 3)), 'b')
ylabel('Pitch')

subplot(223)
plot(Time, atan2(Imu_Rot(:, 2), Imu_Rot(:, 1)), 'b')
ylabel('Yaw')

%%
figure(24)
subplot(221)
plot(Time, Val_P_Z)
ylabel('VAL_P_Z')

subplot(222)
plot(Time, Val_I_Z)
ylabel('VAL_I_Z')

subplot(223)
plot(Time, Val_D_Z)
ylabel('VAL_D_Z')


figure(25)
subplot(221)
plot(Time, Imu_AccX, 'b');
ylabel('Ax');

subplot(222)
plot(Time, Imu_AccY, 'b');
ylabel('Ay');

subplot(223);
plot(Time, Imu_AccZ, 'b');
ylabel('Az');

figure(100)
hold on;
plot(Time, Lidar_Dist, 'k');
plot(Time, -Filter_Z, '--r');
hold off;
ylabel('Z Filter')
xlabel('Time')

figure(101)
hold on;
plot(Time(2:end), lidarTime(2:end), 'g');
ylabel('Latest Lidar Timestamp');
xlabel('Time');
hold off;

figure(88)
hold on;
plot(Time(2:end), RCinError(2:end), 'g');
xlabel('Time');
ylabel('Error Code');
hold off;

%%
Real_Time_for_this_log = (Time(end) - Time(1)) / 60
