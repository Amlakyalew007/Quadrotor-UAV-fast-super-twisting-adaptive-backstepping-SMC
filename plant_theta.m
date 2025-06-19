function y = plant_theta(in)
% System parameters
    m = 0.5;
    d = 0.3;
    ob = 1;
    g = 9.8;
    Ix = 0.006; Iy = 0.006; Iz = 0.012; 
    Jr =5e-5;

    % System dynamics coefficients
    a1 = (Iy - Iz) / Ix;
    a2 = -Jr / Ix;
    a3 = (Iz - Ix) / Iy;
    a4 = Jr / Iy;
    a5 = (Ix - Iy) / Iz;
    b1 = d / Ix;
    b2 = d / Iy;
    b3 = 1 / Iz;

%%%%%%%%%%%%%%%%%%%%%%%
%   INPUTS    % 
%%%%%%%%%%%%%%%%%%%%%%%

x3 = in(1); % x3 is the angular position (theta) 
x4 = in(2); % x4 is the angular velocity (q)
d2 = in(3); % disturbance
U3 = in(4); % control input
x2 = in(5);
x6 = in(6);


% theta sub-system of the quadrotor UAV (state space representation)

dx3 = x4;
dx4 = a3 * x2 * x6 + a4 * ob * x2 + b2 * U3;


y  =  [dx3, dx4]';
