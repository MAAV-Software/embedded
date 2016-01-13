close all
clc

log = load('~/maav/ctrl/test/bin/LKLOG57.TXT');

% parsing the LogKalman Ouput
Time          = log(:,1);
LidarDist     = log(:,2);
xFilt         = log(:,3);
yFilt         = log(:,4);
zFilt         = log(:,5);
xdotFilt      = log(:,6);
ydotFilt      = log(:,7);
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
