function [x, P] = update(x_old, P_old, u, Q, deltaT);

A = zeros(6, 6);
A(1, 1) = 1;
A(1, 2) = deltaT;
A(2, 2) = 1;
A(3, 3) = 1;
A(3, 4) = deltaT;
A(4, 4) = 1;
A(5, 5) = 1;
A(5, 6) = deltaT;
A(6, 6) = 1;

B = zeros(6, 3);
B(2, 1) = deltaT;
B(4, 2) = deltaT;
B(6, 3) = deltaT;

x = A * x_old + B * u;
P = A * P_old * A' + Q * deltaT;


