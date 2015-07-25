function []=quadplots(t_quad,X_quad,EBwa,TBwa,UBw)
%This function m-file plots the results of the ODE solver after integrating 
%the ODE system. 

figure(1)
plot(t_quad,X_quad(:,1),t_quad,X_quad(:,2),'--g',t_quad,X_quad(:,3),'.-r')
title('Position of the Quad-Rotor Helicopter Vs. Time, Frame F_a')
xlabel('Time (Seconds)')
ylabel('Position (m)')
legend('X-Position','Y-Position','Z-Position')

figure(2)
plot3(X_quad(1,1),X_quad(1,2),X_quad(1,3),'*g')
hold on
plot3(X_quad(2:numel(t_quad)-1,1),X_quad(2:numel(t_quad)-1,2),X_quad(2:numel(t_quad)-1,3))
hold on
plot3(X_quad(numel(t_quad),1),X_quad(numel(t_quad),2),X_quad(numel(t_quad),3),'*r')
title('Trajectory of the Quad-Rotor Helicopter, Frame F_a')
xlabel('X-Position (m)')
ylabel('Y-Position (m)')
zlabel('Z-Position (m)')
legend('Start','Path','End')
grid on

figure(3)
plot(t_quad,X_quad(:,4),t_quad,X_quad(:,5),'--g',t_quad,X_quad(:,6),'.-r')
title('Velocity of the Quad-Rotor Helicopter Vs. Time, Frame F_a')
xlabel('Time (Seconds)')
ylabel('Velocity (m/s)')
legend('X-Speed','Y-Speed','Z-Speed')

figure(4)
plot(t_quad,(180/pi)*X_quad(:,7),t_quad,(180/pi)*X_quad(:,8),'--g',t_quad,(180/pi)*X_quad(:,9),'.-r')
title('Angular Orientation of the Quad-Rotor Helicopter Vs. Time')
xlabel('Time (Seconds)')
ylabel('Anglular Orientation (Degrees)')
legend('Pitch Angle','Roll Angle','Yaw Angle')

figure(5)
plot(t_quad,(180/pi)*X_quad(:,10))
title('Angular Positions of the Four Rotors Vs. Time')
xlabel('Time (Seconds)')
ylabel('Anglular Positions (Degrees)')

figure(6)
plot(t_quad,(180/pi)*X_quad(:,11),t_quad,(180/pi)*X_quad(:,12),'--g',t_quad,(180/pi)*X_quad(:,13),'.-r')
title('Angular Velocity \omega^b^a_b Components Vs. Time, Frame F_b')
xlabel('Time (Seconds)')
ylabel('Angular Velocity \omega^b^a_b Components (Degrees)')
legend('\omega^b^a_b1','\omega^b^a_b2','\omega^b^a_b3')

figure(7)
plot(t_quad,EBwa,t_quad,TBwa,'--g',t_quad,UBw,'.-r')
title('Quad-Rotor Helicopter Energy Vs. Time')
xlabel('Time (Seconds)')
ylabel('Energy (J)')
legend('Total Energy','Kinetic Energy','Potential Energy')

end