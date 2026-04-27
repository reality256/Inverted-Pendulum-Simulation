M = 1.0;
m = 0.1;
l = 0.5;
g = 9.8;

% Linearized state-space model around upright position
% State: X = [x; x_dot; theta; theta_dot]
A = [0 1 0 0;
     0 0 -m*g/M 0;
     0 0 0 1;
     0 0 (M+m)*g/(M*l) 0];

B = [0;
     1/M;
     0;
     -1/(M*l)];

% LQR weights
Q = diag([10, 1, 200, 10]);
R = 0.1;

% LQR gain
K = lqr(A, B, Q, R);

% Force saturation
Fmax = 50;

disp("LQR gain K:");
disp(K);