function [linearized_system] = linearize_system(curr_state, ctrl_input, mass)
% takes in current state, control input and mass and generates a linearized
% system around that state and control input
x = curr_state(1);
y = curr_state(2);
z = curr_state(3);
xd = curr_state(4);
yd = curr_state(5);
zd = curr_state(6);
r = curr_state(7);
p = curr_state(8);
yaw = curr_state(9);
ctrl_fz = ctrl_input(1);
ctrl_r = ctrl_input(2);
ctrl_p = ctrl_input(3);
ctrl_yawd = ctrl_input(4);

linearized_system = zeros(size(curr_state, 1), size(curr_state, 1));
linearized_system(1, 4) = 1;
linearized_system(2, 5) = 1;
linearized_system(3, 6) = 1;
linearized_system(4, 8) = -cos(p) * ctrl_fz / mass;
linearized_system(5, 7) = cos(r) * cos(p) * ctrl_fz / mass;
linearized_system(5, 8) = -sin(r) * sin(p) * ctrl_fz / mass;
linearized_system(6, 7) = -sin(r) * cos(p) * ctrl_fz / mass;
linearized_system(6, 8) = -cos(r) * sin(p) * ctrl_fz / mass;

