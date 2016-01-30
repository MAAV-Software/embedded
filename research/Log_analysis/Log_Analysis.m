close all
clc
%clf
log = load('futaba_vs_hitech/LOG85.TXT');

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
AtomFlag          = log(:,85);
DJI_Roll_RAW      = log(:,86);
DJI_Pitch_RAW     = log(:,87);
DJI_Fz_RAW    = log(:,88);
DJI_YawRate_RAW        = log(:,89);

PPM_1 = log(:, 90);
PPM_2 = log(:, 91);
PPM_3 = log(:, 92);
PPM_4 = log(:, 93);

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
plot(Time, Filter_Yaw, 'b', Time, Setpt_Yaw, 'g')
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
plot(Time, DJI_Roll, 'b')
xlabel('Time');
ylabel('DJI Roll PWM');
subplot(222)
plot(Time, DJI_Pitch, 'b')
xlabel('Time');
ylabel('DJI Pitch PWM');
subplot(223)
plot(Time, DJI_Yawdot, 'b')
xlabel('Time');
ylabel('DJI Yaw Dot PWM');
subplot(224)
plot(Time, DJI_Fz, 'b')
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


figure(22)
plot(Time, PPM_3, 'b');
ylabel('Chan 3 PPM')

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
plot(Time, Lidar_Dist, 'g');
plot(Time, -Filter_Z, '--b');
hold off;
ylabel('Z Filter')
xlabel('Time')

%%
Real_Time_for_this_log = (Time(end) - Time(1)) / 60