function y = plant_psi(in)
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

x5 = in(1); % x5 is the angular position (psi) 
x6 = in(2); % x6 is the angular velocity (r)
d2 = in(3); % disturbance
U4 = in(4); % control input
x2 = in(5);
x4 = in(6);


% psi sub-system of the quadrotor UAV (state space representation)

dx5 = x6;
dx6 = a5 * x2 * x4 + b3 * U4 + d2;


y  =  [dx5, dx6]';
