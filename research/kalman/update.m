function [next_state, out_P] = update(curr_state, P, R, sensor)
% does update step of filter
linearized_mat = linearize_sensor(curr_state);
L = P * linearized_mat' * inv(R + linearized_mat * P * linearized_mat');
out_P = (eye(size(P, 1)) - L * linearized_mat) * P;

error = sensor - sensor_func(curr_state);
next_state = curr_state + L * error;

