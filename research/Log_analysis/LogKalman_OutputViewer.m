close all
clc

log = load('TEMP.TXT');

% parsing the LogKalman Ouput
Time          = log(:,1);
LidarDist     = log(:,2);
xFilt         = log(:,3);
yFilt         = log(:,5);
zFilt         = log(:,7);
xdotFilt      = log(:,4);
ydotFilt      = log(:,6);
zdotFilt      = log(:,8);


clf

%% plot
% figure 1 zFilt vs LidarDist
figure(1)
hold on
plot(Time, zFilt, 'b')
plot(Time, LidarDist, 'g')
hold off
xlabel('Time')
ylabel('Value Z')
