function [next_state, out_P] = predict(curr_state, ctrl_input, P, Q, G, deltaTime, mass);
% does prediction step of filter

next_state = sys_func(curr_state, ctrl_input) * deltaTime + curr_state;
linearized_model = linearize_system(curr_state, ctrl_input, mass); % is a matrix
out_P = P + deltaTime * (linearized_model * P + P * linearized_model' + G * Q * G');
