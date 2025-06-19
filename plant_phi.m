function y = plant_phi(in)
% System parameters
m = 0.5;
    d = 0.3;
    ob = 1;
    g = 9.8;
    Ix = 0.006; Iy = 0.006; Iz = 0.012; 
    % Kfax = 5.567e-4; Kfay = 5.567e-4; Kfaz = 6.354e-4;
    % Kftx = 5.567e-4; Kfty = 5.567e-4; Kftz = 6.354e-4; 
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

x1 = in(1); % x1 is the angular position (phi) 
x2 = in(2); % x2 is the angular velocity (p)
d2 = in(3); % disturbance
U2 = in(4); % control input
x4 = in(5);
x6 = in(6);


% phi sub-system of the quadrotor UAV (state space representation)

dx1 = x2;
dx2 = a1 * x4 * x6 + a2 * ob * x4 + b1 * U2 + d2;


y  =  [dx1, dx2]';
