function y = plant_z(in)
% System parameters
m = 0.5;
g = 9.8;

%%%%%%%%%%%%%%%%%%%%%%%
%   INPUTS    % 
%%%%%%%%%%%%%%%%%%%%%%%
x11 = in(1); % x11 is the position along Z-axis
x12 = in(2); % x12 is the velocity along Z-axis
d2 = in(3); % disturbance
U1 = in(4); 
x1 = in(5);
x3 = in(6);


% Z sub-system of the quadrotor UAV (state space representation) 

dx11 = x12;
dx12 = (cos(x1) * cos(x3) * U1) / m - g + d2;


y  =  [dx11, dx12]';
