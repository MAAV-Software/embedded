P = eye(9, 9); 
R = eye(6, 6); % sensor covariances
Q = eye(9, 9); % motion covariances
G = eye(9, 9); % idk what this is

mass = 0.3; % kg
deltaTime = 0.1; % seconds
iterations = 10;

state = zeros(9, 1);
input = [10, 0.5, 0, 0]';
sensor = [0.2, 0, 0, 0, 1, 10]';

past_states(:, 1) = state;

for i = 1:iterations
    [state, P] = predict(state, input, P, Q, G, deltaTime, mass);
    [state, P] = update(state, P, R, sensor); 
    state = substitute_rpy(state, sensor);
    past_states(:, i + 1) = state;
end

time = linspace(0, deltaTime * iterations, iterations + 1);
plot(time, past_states(3, :));
