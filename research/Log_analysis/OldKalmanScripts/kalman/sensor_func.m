function [predicted_sensor] = sensor_func(curr_state)
% nonlinear function mapping current state to sensor measurements
x = curr_state(1);
y = curr_state(2);
z = curr_state(3);
xd = curr_state(4);
yd = curr_state(5);
zd = curr_state(6);
r = curr_state(7);
p = curr_state(8);
yaw = curr_state(9);

predicted_sensor(1) = r;
predicted_sensor(2) = p;
predicted_sensor(3) = yaw;
predicted_sensor(4) = xd * cos(yaw);
predicted_sensor(5) = xd * sin(yaw);
predicted_sensor(6) = z / (cos(r) * cos(p));

predicted_sensor = predicted_sensor';