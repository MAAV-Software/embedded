log = load('rclog/LOG1.TXT');

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
DJI_Yawdot_RAW    = log(:,88);
DJI_Fz_RAW        = log(:,89);

%% plot
% figure 1 values
figure(1)
subplot(221)
xlabel('Time')
ylabel('Value X')
plot(Time, Filter_X, 'b', Time, Setpt_X, 'g')
subplot(222)
xlabel('Time')
ylabel('Value Y')
plot(Time, Filter_Y, 'b', Time, Setpt_Y, 'g')
subplot(223)
xlabel('Time')
ylabel('Value Z')
plot(Time, Filter_Z, 'b', Time, Setpt_Z, 'g')
subplot(224)
xlabel('Time')
ylabel('Value Yaw')
plot(Time, Filter_Yaw, 'b', Time, Setpt_Yaw, 'g')

% figure 2 rates
figure(2)
subplot(221)
xlabel('Time')
ylabel('Rate X')
plot(Time, Filter_Xdot, 'b', Time, Setpt_Xdot, 'g')
subplot(222)
xlabel('Time')
ylabel('Rate Y')
plot(Time, Filter_Ydot, 'b', Time, Setpt_Ydot, 'g')
subplot(223)
xlabel('Time')
ylabel('Rate Z')
plot(Time, Filter_Zdot, 'b', Time, Setpt_Zdot, 'g')

% figure 3 DJI
figure(3)
subplot(221)
xlabel('Time')
ylabel('DJI Roll')
plot(Time, DJI_Roll, 'b')
subplot(222)
xlabel('Time')
ylabel('DJI Pitch')
plot(Time, DJI_Roll, 'b')
subplot(223)
xlabel('Time')
ylabel('DJI Yaw Dot')
plot(Time, DJI_Yawdot, 'b')
subplot(224)
xlabel('Time')
ylabel('DJI Fz')
plot(Time, DJI_Fz, 'b')

% figure 4 others
figure(4)
subplot(221)
xlabel('Time')
ylabel('Battery')
plot(Time, Battery, 'b')


% figure 3 DJI
figure(5)
subplot(221)
xlabel('Time')
ylabel('DJI Roll Raw')
plot(Time, DJI_Roll_RAW, 'b')
subplot(222)
xlabel('Time')
ylabel('DJI Pitch Raw')
plot(Time, DJI_Roll_RAW, 'b')
subplot(223)
xlabel('Time')
ylabel('DJI Yaw Dot Raw')
plot(Time, DJI_Yawdot_RAW, 'b')
subplot(224)
xlabel('Time')
ylabel('DJI Fz Raw')
plot(Time, DJI_Fz_RAW, 'b')





