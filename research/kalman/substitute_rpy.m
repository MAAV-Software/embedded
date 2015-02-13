function [next_state] = substitute_rpy(state, sensor)
% substitutes the rpy of the sensor into the state
next_state = state;
next_state(7) = sensor(1);
next_state(8) = sensor(2);
next_state(9) = sensor(3);
