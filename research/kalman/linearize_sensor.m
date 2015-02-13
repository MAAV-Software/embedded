function [linearized_sensor] = linearize_system(curr_state)
% Takes in a current state and generates a linearized sensor model around
% that state
x = curr_state(1);
y = curr_state(2);
z = curr_state(3);
xd = curr_state(4);
yd = curr_state(5);
zd = curr_state(6);
r = curr_state(7);
p = curr_state(8);
yaw = curr_state(9);

linearized_sensor = zeros(6, 9);
linearized_sensor(1, 7) = 1;
linearized_sensor(2, 8) = 1;
linearized_sensor(3, 9) = 1;
linearized_sensor(4, 4) = cos(yaw);
linearized_sensor(4, 9) = -xd * sin(yaw);
linearized_sensor(5, 4) = sin(yaw);
linearized_sensor(5, 9) = xd * cos(yaw);
linearized_sensor(6, 3) = 1 / (cos(r) * cos(p));
linearized_sensor(6, 7) = z * sin(r) / (cos(p) * cos(r) * cos(r));
linearized_sensor(6, 8) = z * sin(p) / (cos(r) * cos(p) * cos(p));


