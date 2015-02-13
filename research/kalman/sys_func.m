function [delta_state] = sys_func(curr_state, ctrl_input)
% nonlinear function that describes state derivative as function of current
% state and input
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


delta_state(1) = xd;
delta_state(2) = yd;
delta_state(3) = zd;

roll_mat = [ 1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r) ];
pitch_mat = [ cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p) ];
yaw_mat = [ cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1 ];
rpy_mat = yaw_mat * pitch_mat * roll_mat;

x_dir = [1 0 0];
y_dir = [0 1 0];
z_dir = [0 0 1];

fz_inertial = rpy_mat' * [0 0 ctrl_fz]';

delta_state(4) = fz_inertial(1);
delta_state(5) = fz_inertial(2);
delta_state(6) = fz_inertial(3) - 9.8;

delta_state(7) = 0;
delta_state(8) = 0;
delta_state(9) = ctrl_yawd;
delta_state = delta_state';



