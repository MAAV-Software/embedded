x_old = zeros(6,1);
P_old = zeros(6,6);
u = [0.01; 1; 0.001];
dt = 0.01;
Q = zeros(6,6);


[x, P] = update(x_old, P_old, u, Q, dt);
x_old = x
P_old = P

R_flow = zeros(2,2);
z_flow = [0.5; 1.0];
[x, P] = correct_optical_flow(x_old, P_old, z_flow, R_flow);
x
P