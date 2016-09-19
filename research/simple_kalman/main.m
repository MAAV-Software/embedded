function kalmanMain(filename)

clear all;
close all;

% log reading
%log = load('RunFilterTestLog.TXT');
%log = load('../Log_analysis/23feb2016/LOG94.TXT');
log = load(filename)

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

lidarTime     = log(:,95);
px4Time       = log(:,96);

last_imu_time = -1;
last_time = 0;
last_imu = [0, 0, 0]';
last_lidar_time = 0;
last_lidar = 0;
last_px4_time = 0;
last_px4 = [0, 0]';

% initial states
x = [0, 0, 0, 0, 0, 0]';
P = zeros(6);
%P = diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]);

% noise matrices
%Q = diag([0.5, 0.1, 0.5, 0.1, 0.1, 0.1]); % Original Clark Values
Q = diag([0.5, 0.1, 0.5, 0.1, 0.01^2, 0.005^2]); % New Values for z zd
%R_lidar = diag([0.025, 0.05]); % Original Clark Values
R_lidar = diag([0.0009, 0.01]); % New values
R_optical_flow = diag([.8, 0.8]);

for i = 1:log_length
    time = Time(i);
    
    % rotation matrix that turns vectors in quadcopter frame to earth frame
    rotMat = reshape(Imu_Rot(i, :), [3, 3]);
    %rotMat = rotMat';
    
    % rotating imu
    imu = [Imu_AccX(i), Imu_AccY(i), Imu_AccZ(i)]';
    imu = rotMat * imu;
    imu(3) = imu(3) + 9.81;
    
    % rotating lidar
    lidar = rotMat * [0 0 -Lidar_Dist(i)]';
    lidar = lidar(3);
    
    % rotating px4 only for yaw
    px4RotMat = [cos(Yaw(i)), sin(Yaw(i)); -sin(Yaw(i)), cos(Yaw(i))];
    px4 = px4RotMat * [Px4_Xdot(i), Px4_Ydot(i)]';
    
    if (last_imu_time == -1) 
       last_imu_time = time;
       last_time = time;
       continue;
    end
    
    % update step
    if (Time(i) ~= last_time) 
        last_imu = imu;
        deltaT = Time(i) - last_time;
        last_time = Time(i);
        %imu = [0,0,0]';
        [x, P] = update(x, P, imu, Q, deltaT);
    end

 
    % correct lidar
    if (lidarTime(i) ~= last_lidar_time)
        del_height = lidar - last_lidar;
        lidar_meas = [lidar, del_height]';
        [x, P] = correct_lidar(x, P, lidar_meas, R_lidar);
        last_lidar_time = lidarTime(i);
        last_lidar = lidar;
    end
    
    % correct optical flow
    if (last_px4_time ~= px4Time(i)) 
        last_px4 = px4;
        last_px4_time = px4Time(i);
        [x, P] = correct_optical_flow(x, P, last_px4, R_optical_flow);
    end

    
    lidar_hist(:, i) = lidar;
    x_hist(:, i) = x;
    P_hist(:, :, i) = P;
    px4_hist(:, :, i) = px4;
end

%%
figure(1)
hold on;
heightVar = 3 * P_hist(5, 5, :);
heightVar = heightVar(:);
upperVar = -x_hist(5, :) + sqrt(heightVar');
lowerVar = -x_hist(5, :) - sqrt(heightVar');
plot(Time,-x_hist(5, :), Time, -lidar_hist, Time, upperVar, Time, lowerVar);
%legend('z');
hold off;
%legend('filtered z', 'lidar rotated', 'upper var', 'lower var');

figure(2)
plot(Time, -x_hist(6, :));
%legend('z rate');

figure(3)
px4_x = px4_hist(1, 1, :);
plot(Time, x_hist(2, :), Time, px4_x(:), '*');
%legend('filtered x rate', 'px4 x rate');

figure(4)
px4_y = px4_hist(2, 1, :);
plot(Time, x_hist(4, :), Time, px4_y(:), '*');
%legend('filtered y rate', 'px4 y rate');

figure(5)
plot(Time, x_hist(1, :));
%legend('filtered x');

figure(6)
plot(Time, x_hist(3, :));
%legend('filtered y');

figure(7)

%x_save = x_hist';
%save('runFltAns2.txt','x_save','-ascii','-double','-tabs');
