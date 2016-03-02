x = [0, 0, 0, 0, 0, 0]';
P = zeros(6);


u = [0.1, 0.2, 0.3]';
Q = diag([0.5, 0.1, 0.5, 0.1, 0.1, 0.1]);
dt = 0.01;
[x, P] = update(x, P, u, Q, dt);
[x, P] = update(x, P, u, Q, dt);
[x, P] = correct
