clear;
clc;
close all;


log = load('../Log_analysis/23feb2016/LOG94.TXT');
Time          = log(:,1);
Imu_AccX      = log(:,3);
Imu_AccY      = log(:,4);
Imu_AccZ      = log(:,5);
Imu_Rot       = log(:,12:20);

Yaw           = atan2(Imu_Rot(:, 2), Imu_Rot(:, 1));

Px4_Xdot      = log(:,21); % meters / second
Px4_Ydot      = log(:,22); % meters / second
Px4_Qual      = log(:,23);
Lidar_Dist    = log(:,24); % meters

log_length = size(Time, 1);


% imu vectors are only of accels in order of (ax, ay, az)
imu_flt = zeros(3,1);

fc = 1;
dt = 0.012;
%alpha = 0.2;

for i = 1:log_length
    % rotation matrix that turns vectors in quadcopter frame to earth frame
    rotMat = reshape(Imu_Rot(i, :), [3, 3]);
    
    % rotating imu
    imu = [Imu_AccX(i), Imu_AccY(i), Imu_AccZ(i)]';
    imu = rotMat * imu;
    imu(3) = imu(3) + 9.81;
    
    if (i ~= 1) 
        dt = Time(i) - Time(i-1); 
    end
    
    alpha = (2.0 * pi * dt * fc) / ((2.0 * pi * dt * fc) + 1.0);
    
    imu_flt = (alpha * imu) + ((1 - alpha) * imu_flt);
    imu_flt_hist(:,i) = imu_flt;
    imu_hist(:,i) = imu;
    Time_hist(:,i) = Time(i);
end

figure(1)
plot(Time_hist, imu_hist(1,:), '--k', Time_hist, imu_flt_hist(1,:), 'r');
ylabel('Accel (m/s^2)');
xlabel('Time (s)');
title('AccX Flt (red) & AccX Raw (blk)');

figure(2)
plot(Time_hist, imu_hist(2, :), '--k', Time_hist, imu_flt_hist(2,:), 'r');
ylabel('Accel (m/s^2)');
xlabel('Time (s)');
title('AccY Flt (red) & AccY Raw (blk)');

figure(3)
plot(Time_hist, imu_hist(3, :), '--k', Time_hist, imu_flt_hist(3,:), 'r');
ylabel('Accel (m/s^2)');
xlabel('Time (s)');
title('AccZ Flt (red) & AccZ Raw (blk)');
