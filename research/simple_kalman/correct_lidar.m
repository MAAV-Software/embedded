function [x, P] = correct_lidar(x_old, P_old, z, R);

H = [0 0 0 0 1 0; 0 0 0 0 0 1];
y = z - H * x_old;

S = H * P_old * H' + R;
K = P_old * H' * inv(S);

x = x_old + K * y;
P = (eye(6) - K * H) * P_old;


