%%  manually load data
close all
clc
clear

load('log.txt')

AccX = log(:,1);
AccY = log(:,2);
AccZ = log(:,3);
AngRateX = log(:,4);
AngRateY = log(:,5);
AngRateZ = log(:,6);
Roll = log(:,7);
Pitch = log(:,8);
Yaw = log(:,9);
t = 1:1:length(AccX);
t = t';

linewidth = 2;

figure(1)
plot(t, AccX, 'b', 'LineWidth', linewidth, t, AccY, 'r', 'LineWidth', linewidth, t, AccZ, 'g', 'LineWidth', linewidth)
legend('AccX','AccY','AccZ');
title('Acceleration, (g)')
grid on

figure(2)
plot(t, AngRateX, 'b', 'LineWidth', linewidth, t, AngRateY, 'r', 'LineWidth', linewidth, t, AngRateZ, 'g', 'LineWidth', linewidth)
legend('AngRateX','AngRateY','AngRateZ');
title('AngRate (rad/s)')
grid on

figure(3)
plot(t, Roll,'b', 'LineWidth', linewidth, t, Pitch,'r', 'LineWidth', linewidth, t, Yaw, 'g', 'LineWidth', linewidth)
legend('Roll','Pitch','Yaw');
title('Euler Angle (rad)')
grid on

muAccX = mean(AccX)
muAccY = mean(AccY)  
muAccZ = mean(AccZ)    
sigmaAccX = std(AccX)
sigmaAccY = std(AccY)
sigmaAccZ = std(AccZ)
varAccX = sigmaAccX^2
varAccY = sigmaAccY^2
varAccZ = sigmaAccZ^2
disp('------------------------------------')

muAngRateX = mean(AngRateX)
muAngRateY = mean(AngRateY)
muAngRateZ = mean(AngRateZ)
sigmaAngRateX = std(AngRateX)
sigmaAngRateY = std(AngRateY)
sigmaAngRateZ = std(AngRateZ)
varAngRateX = sigmaAngRateX^2
varAngRateY = sigmaAngRateY^2
varAngRateZ = sigmaAngRateZ^2
disp('------------------------------------')

muRoll = mean(Roll)
muPitch = mean(Pitch)
muYaw = mean(Yaw)
sigmaRoll = std(Roll)
sigmaPitch = std(Pitch)
sigmaYaw = std(Yaw)
varRoll = sigmaRoll^2
varPitch = sigmaPitch^2
varYaw = sigmaYaw^2
disp('------------------------------------')
